/*
 * BotatoMms — runs BOTATO's maze logic inside mms (mackorone Micromouse
 * Simulator).
 *
 * This file is only a loader. The algorithm is compiled straight from the
 * firmware tree, unmodified:
 *
 *   Core/App/floodfill/FloodFill.cpp/.hpp   flood fill (BFS distances, nextDir)
 *   Core/App/maze/MazeMap.cpp/.hpp          wall map
 *   Core/App/maze/MazeConfig.h              size, start, goal box
 *   Core/App/planner/Planner.cpp/.hpp       pose + turn translation
 *
 * Edit any of those and rebuild in mms: the simulator runs the new code.
 *
 * What lives here instead of the firmware:
 *   Body        — mms stands in for the motors and ToF sensors
 *   Display     — paints walls, distances and the path in the mms window
 *   GridNavigator — the cell-level decisions of Navigator.cpp (senseWalls,
 *                 decide, act, start-corner detection, homing). Navigator.cpp
 *                 itself can't be compiled here (it drives ControlLoop,
 *                 encoders, ToF), so this is a line-for-line twin of its grid
 *                 logic with the motion control left out. If you change the
 *                 POLICY in Navigator::decide()/act()/senseWalls(), mirror it
 *                 in the marked functions below. FloodFill changes need nothing.
 *
 * Run options (append to the mms "Run Command"):
 *   --mirror        present the mms maze mirrored, so the robot starts with
 *                   the maze on its LEFT (ROBOFEST 2026 layout, matches
 *                   MazeConfig START_X = WIDTH-1 with no corner detection).
 *                   Without it the robot starts with the maze on its right and
 *                   the firmware's start-corner detection has to find that.
 *   --speed-runs N  speed runs after the search (default 1)
 *   --no-text       don't draw flood distances (faster)
 *
 * mms Reset button = lifting the robot back to the start: the map is kept;
 * if the search had reached the goal the next run is a speed run, otherwise a
 * fresh search (same as the firmware's buttons).
 */
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>

#include "API.h"
#include "MazeConfig.h"
#include "MazeMap.hpp"
#include "FloodFill.hpp"
#include "Planner.hpp"

namespace {

struct Options {
    bool mirror    = false;
    int  speedRuns = 1;
    bool text      = true;
};

/* ---- Body: the simulator in place of motors and sensors ------------------
 * Robot-relative only, so it doesn't care which frame the map is in. Mirror
 * swaps left and right: the robot then lives in the mirror image of the mms
 * maze. */
class Body {
public:
    explicit Body(bool mirror) : mirror_(mirror) {}
    bool wallLeft()  const { return mirror_ ? API::wallRight() : API::wallLeft(); }
    bool wallRight() const { return mirror_ ? API::wallLeft()  : API::wallRight(); }
    bool wallFront() const { return API::wallFront(); }
    void turnLeft()        { if (mirror_) API::turnRight(); else API::turnLeft(); }
    void turnRight()       { if (mirror_) API::turnLeft();  else API::turnRight(); }
    bool forward()         { return API::moveForward(); }
private:
    bool mirror_;
};

/* ---- Display: firmware map -> mms window ---------------------------------
 * The robot physically starts in mms cell (0,0) facing north. In the firmware
 * frame it starts at (startX, START_Y). x offsets from the start column map
 * straight across, negated when mirrored. */
class Display {
public:
    Display(bool mirror, bool text, int w, int h)
        : mirror_(mirror), text_(text), w_(w), h_(h) {}

    void setStartX(int sx) { startX_ = sx; }

    bool toMms(int fx, int fy, int& mx, int& my) const {
        const int dx = fx - startX_;
        mx = mirror_ ? -dx : dx;
        my = fy - maze::START_Y;
        return mx >= 0 && mx < w_ && my >= 0 && my < h_;
    }

    char dirChar(Dir d) const {
        static const char c[4] = { 'n', 'e', 's', 'w' };
        if (mirror_ && (d == EAST || d == WEST)) d = MazeMap::opposite(d);
        return c[d];
    }

    void walls(const MazeMap& m, int fx, int fy) const {
        int mx, my;
        if (!toMms(fx, fy, mx, my)) return;
        for (int d = 0; d < 4; ++d)
            if (m.hasWall(fx, fy, static_cast<Dir>(d)))
                API::setWall(mx, my, dirChar(static_cast<Dir>(d)));
    }

    void cell(int fx, int fy, char color) const {
        int mx, my;
        if (toMms(fx, fy, mx, my)) API::setColor(mx, my, color);
    }

    // Flood distances. Only cells whose text changed are re-sent.
    void distances(const FloodFill& f) {
        if (!text_) return;
        for (int fx = 0; fx < maze::WIDTH; ++fx)
            for (int fy = 0; fy < maze::HEIGHT; ++fy) {
                int mx, my;
                if (!toMms(fx, fy, mx, my)) continue;
                const uint16_t d = f.distance(fx, fy);
                const std::string t = (d == FloodFill::INF) ? std::string() : std::to_string(d);
                if (t == shown_[mx][my]) continue;
                shown_[mx][my] = t;
                if (t.empty()) API::clearText(mx, my); else API::setText(mx, my, t);
            }
    }

    // Frame changed (start corner found) or a new search: start over.
    void redraw(const MazeMap& m) {
        API::clearAllColor();
        API::clearAllText();
        for (auto& col : shown_) for (auto& s : col) s.clear();
        for (int fx = 0; fx < maze::WIDTH; ++fx)
            for (int fy = 0; fy < maze::HEIGHT; ++fy)
                if (m.isVisited(fx, fy)) walls(m, fx, fy);
    }

private:
    bool mirror_, text_;
    int  w_, h_;
    int  startX_ = maze::START_X;
    std::string shown_[maze::MAX_DIM][maze::MAX_DIM];
};

/* ---- GridNavigator: Navigator.cpp's cell-level logic ---------------------- */
class GridNavigator {
public:
    enum State   : uint8_t { IDLE, SEARCH, RETURN, SPEED, DONE, STUCK };
    enum Action  : uint8_t { ACT_STRAIGHT, ACT_LEFT, ACT_RIGHT, ACT_AROUND,
                             ACT_GOAL, ACT_HOME, ACT_PARKED, ACT_STUCK };
    enum Outcome : uint8_t { OUT_DONE, OUT_STUCK, OUT_CRASH, OUT_RESET };

    GridNavigator(Body& body, Display& disp) : body_(body), disp_(disp) { clearMap(); }

    bool mapReady() const { return mapReady_; }

    // Navigator::startSearch()
    Outcome startSearch() {
        clearMap();                     // search always starts from the assumption
        disp_.setStartX(startX_);
        disp_.redraw(map_);
        return run(SEARCH);
    }

    // Navigator::startSpeed()
    Outcome startSpeed() {
        flood_.toGoal(map_, true);
        API::clearAllColor();
        const uint16_t len = flood_.distance(startX_, maze::START_Y);
        API::log("  known path start->goal: " + std::to_string(len) + " cells");
        return run(SPEED);
    }

private:
    Body&     body_;
    Display&  disp_;
    MazeMap   map_;
    FloodFill flood_;
    GridPose  pose_;

    State state_       = IDLE;
    bool  mapReady_    = false;
    bool  otherSide_   = false;
    bool  returnKnown_ = false;
    int   startX_      = maze::START_X;

    // per-leg stats
    int cells_ = 0, turns_ = 0, decisions_ = 0;

    static const char* name(State s) {
        switch (s) {
            case SEARCH: return "SEARCH"; case RETURN: return "RETURN";
            case SPEED:  return "SPEED";  case DONE:   return "DONE";
            case STUCK:  return "STUCK";  default:     return "IDLE";
        }
    }

    char trailColor() const {
        if (state_ == SPEED) return 'G';
        if (state_ == RETURN) return returnKnown_ ? 'g' : 'c';
        return 'B';
    }

    void logLeg(const char* what) {
        API::log(std::string("  ") + what + ": " + std::to_string(cells_) + " cells, " +
                 std::to_string(turns_) + " turns, " + std::to_string(decisions_) + " decisions");
        cells_ = turns_ = decisions_ = 0;
    }

    // Navigator::clearMap()
    void clearMap() {
        map_.reset();
        mapReady_  = false;
        otherSide_ = false;
        startX_    = maze::START_X;
    }

    // Navigator::begin() — grid part
    void begin(State s) {
        state_ = s;
        pose_  = GridPose{ startX_, maze::START_Y, NORTH };
        returnKnown_ = false;
        cells_ = turns_ = decisions_ = 0;
        disp_.cell(pose_.x, pose_.y, trailColor());
        API::log(std::string(name(s)) + ": start (" + std::to_string(pose_.x) + "," +
                 std::to_string(pose_.y) + ") facing N");
    }

    // One decision per cell, like Navigator::updateDrive() at the decision point.
    Outcome run(State s) {
        begin(s);
        for (;;) {
            // Speed run and its return trust the map: no sensing.
            if (state_ != SPEED && !returnKnown_) senseWalls();
            const Action a = decide();
            ++decisions_;
            disp_.distances(flood_);
            Outcome out;
            if (act(a, out)) return out;
        }
    }

    // >>> twin of Navigator::boundaryOpen()
    bool boundaryOpen(Dir side, bool sensorOpen) const {
        int nx, ny;
        MazeMap::neighbour(pose_.x, pose_.y, side, nx, ny);
        if (map_.inBounds(nx, ny)) return false;         // not the boundary
        return sensorOpen;
    }

    // >>> twin of Navigator::moveStartToOtherSide()
    void moveStartToOtherSide() {
        const int from = maze::START_X;
        const int to   = maze::WIDTH - 1 - maze::START_X;
        MazeMap old = map_;
        map_.reset();
        for (int y = 0; y < maze::HEIGHT; ++y) {
            if (!old.isVisited(from, y)) continue;
            for (int d = 0; d < 4; ++d)
                if (old.hasWall(from, y, static_cast<Dir>(d))) map_.setWall(to, y, static_cast<Dir>(d));
            map_.markVisited(to, y);
        }
        pose_.x    = to + (pose_.x - from);
        startX_    = to;
        otherSide_ = true;

        API::log("  start corner: maze is on the other side, start column " +
                 std::to_string(from) + " -> " + std::to_string(to));
        disp_.setStartX(startX_);
        disp_.redraw(map_);
    }

    // >>> twin of Navigator::senseWalls()
    void senseWalls() {
        // One sample of each sensor at the decision point.
        const bool wl = body_.wallLeft();
        const bool wf = body_.wallFront();
        const bool wr = body_.wallRight();

        if (!otherSide_ && state_ == SEARCH) {
            const Dir f = pose_.facing;
            if (boundaryOpen(static_cast<Dir>((f + 3) % 4), !wl) ||
                boundaryOpen(static_cast<Dir>((f + 1) % 4), !wr))
                moveStartToOtherSide();
        }

        const Dir f     = pose_.facing;
        const Dir left  = static_cast<Dir>((f + 3) % 4);
        const Dir right = static_cast<Dir>((f + 1) % 4);

        if (wf) map_.setWall(pose_.x, pose_.y, f);
        if (wl) map_.setWall(pose_.x, pose_.y, left);
        if (wr) map_.setWall(pose_.x, pose_.y, right);
        map_.markVisited(pose_.x, pose_.y);

        disp_.walls(map_, pose_.x, pose_.y);
    }

    // >>> twin of Navigator::decide()
    Action decide() {
        if ((state_ == SEARCH || state_ == SPEED) && map_.isGoal(pose_.x, pose_.y))
            return ACT_GOAL;
        if (state_ == RETURN && pose_.x == startX_ && pose_.y == maze::START_Y)
            return ACT_HOME;

        if (state_ == RETURN) flood_.toCell(map_, startX_, maze::START_Y, returnKnown_);
        else                  flood_.toGoal(map_, state_ == SPEED);

        Dir target;
        if (!flood_.nextDir(map_, pose_.x, pose_.y, pose_.facing, target)) return ACT_STUCK;

        switch (Planner::turnFor(pose_.facing, target)) {
            case TURN_LEFT:   return ACT_LEFT;
            case TURN_RIGHT:  return ACT_RIGHT;
            case TURN_AROUND: return ACT_AROUND;
            default:          return ACT_STRAIGHT;
        }
    }

    // >>> twin of Navigator::act(). Returns true when the leg has ended.
    bool act(Action a, Outcome& out) {
        switch (a) {
        case ACT_LEFT:     return turn(TURN_LEFT, out)   || nextCell(out);
        case ACT_RIGHT:    return turn(TURN_RIGHT, out)  || nextCell(out);
        case ACT_AROUND:   return turn(TURN_AROUND, out) || nextCell(out);
        case ACT_STRAIGHT: return nextCell(out);

        case ACT_GOAL: {
            logLeg(state_ == SPEED ? "speed run reached goal" : "search reached goal");
            if (state_ == SEARCH) mapReady_ = true;
            returnKnown_ = (state_ == SPEED);
            state_ = RETURN;
            API::log(std::string("RETURN: ") + (returnKnown_ ? "explored cells only" : "exploring"));
            const Action next = decide();
            ++decisions_;
            disp_.distances(flood_);
            return act(next, out);
        }

        case ACT_HOME:   return startHoming(out);
        case ACT_PARKED: mapReady_ = true; state_ = DONE; out = OUT_DONE; return true;
        case ACT_STUCK:
            API::log("STUCK: no open neighbour with a finite distance at (" +
                     std::to_string(pose_.x) + "," + std::to_string(pose_.y) + ")");
            state_ = STUCK; out = OUT_STUCK; return true;
        }
        return false;
    }

    // Pivot in place (Navigator::startTurn/updateTurn). True = leg ended.
    bool turn(Turn t, Outcome& out) {
        switch (t) {
            case TURN_LEFT:   body_.turnLeft();  break;
            case TURN_RIGHT:  body_.turnRight(); break;
            case TURN_AROUND: body_.turnLeft(); body_.turnLeft(); break;  // +180, anticlockwise
            default: break;
        }
        Planner::applyTurn(pose_, t);
        ++turns_;
        return resetPressed(out);
    }

    // Navigator::nextCell(): drive into the next cell. True = leg ended.
    bool nextCell(Outcome& out) {
        if (!body_.forward()) {
            API::log("CRASH driving " + std::string(1, "NESW"[pose_.facing]) + " from (" +
                     std::to_string(pose_.x) + "," + std::to_string(pose_.y) + ")");
            state_ = STUCK; out = OUT_CRASH; return true;
        }
        Planner::stepForward(pose_);
        ++cells_;
        disp_.cell(pose_.x, pose_.y, trailColor());
        return resetPressed(out);
    }

    // Navigator::startHoming(): face NORTH on the start cell, then park.
    bool startHoming(Outcome& out) {
        const Turn t = Planner::turnFor(pose_.facing, NORTH);
        if (t != TURN_NONE && turn(t, out)) return true;
        logLeg("home");
        return act(ACT_PARKED, out);
    }

    bool resetPressed(Outcome& out) {
        if (!API::wasReset()) return false;
        out = OUT_RESET;
        return true;
    }
};

Options parseArgs(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--mirror"))                        o.mirror = true;
        else if (!std::strcmp(argv[i], "--no-text"))                  o.text = false;
        else if (!std::strcmp(argv[i], "--speed-runs") && i + 1 < argc) o.speedRuns = std::atoi(argv[++i]);
        else API::log(std::string("ignoring unknown option ") + argv[i]);
    }
    return o;
}

} // namespace

int main(int argc, char** argv) {
    const Options opt = parseArgs(argc, argv);

    const int w = API::mazeWidth();
    const int h = API::mazeHeight();
    if (w != maze::WIDTH || h != maze::HEIGHT) {
        API::log("maze is " + std::to_string(w) + "x" + std::to_string(h) +
                 " but MazeConfig.h is " + std::to_string(maze::WIDTH) + "x" +
                 std::to_string(maze::HEIGHT) + " - load a matching maze or edit MazeConfig.h");
        return 1;
    }

    API::log(std::string("BOTATO in mms: maze on the robot's ") +
             (opt.mirror ? "LEFT (mirrored)" : "RIGHT") +
             ", MazeConfig START_X = " + std::to_string(maze::START_X) +
             ", goal x" + std::to_string(maze::GOAL_X_MIN) + "-" + std::to_string(maze::GOAL_X_MAX) +
             " y" + std::to_string(maze::GOAL_Y_MIN) + "-" + std::to_string(maze::GOAL_Y_MAX));

    Body          body(opt.mirror);
    Display       disp(opt.mirror, opt.text, w, h);
    GridNavigator nav(body, disp);

    int speedDone = 0;
    for (;;) {
        // Like the firmware: no map yet -> search; map ready -> speed run.
        const bool speed = nav.mapReady();
        if (speed && speedDone >= opt.speedRuns) break;
        const GridNavigator::Outcome o = speed ? nav.startSpeed() : nav.startSearch();

        if (o == GridNavigator::OUT_RESET) {
            API::ackReset();
            API::log(nav.mapReady() ? "RESET: back at start, map kept -> speed run"
                                    : "RESET: back at start, goal not reached -> new search");
            continue;
        }
        if (o != GridNavigator::OUT_DONE) break;     // stuck or crashed: stop here
        if (speed) ++speedDone;
    }
    API::log("finished");
    return 0;
}
