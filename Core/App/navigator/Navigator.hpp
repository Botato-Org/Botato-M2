/**
 * @file  Navigator.hpp
 * @brief Maze policy: cell-to-cell driving, move decisions, curves, return home and park.
 */
#ifndef APP_NAVIGATOR_HPP
#define APP_NAVIGATOR_HPP

#include <cstdint>
#include "MazeMap.hpp"
#include "FloodFill.hpp"
#include "Planner.hpp"
#include "ControlLoop.hpp"
#include "PIDController.hpp"
#include "WallSensorArray.hpp"
#include "Encoder.hpp"
#include "NavConfig.h"

/**
 * Segment model (DECISIONS #25): drives toward the target cell centre (pose_); segmentEndMm_ is
 * the axle distance at which it gets there. SENSE_LOOKAHEAD_MM short of it, it senses and decides:
 * straight extends the segment, a turn brakes, pivots and drives on. Every run drives home and
 * parks (#31). Speaks only intentions to ControlLoop; never touches hardware. Main loop only.
 */
class Navigator {
public:
    enum State : uint8_t { IDLE, SEARCH, RETURN, SPEED, DONE, STUCK };
    enum Phase : uint8_t { DRIVE, SETTLE, TURN, PARK, FINISHED, ARC };
    enum Action : uint8_t {
        ACT_STRAIGHT,
        ACT_LEFT,
        ACT_RIGHT,
        ACT_AROUND,
        ACT_GOAL,
        ACT_HOME,
        ACT_PARKED,
        ACT_STUCK
    };

    void init(ControlLoop* ctrl, WallSensorArray* walls, Encoder* encL, Encoder* encR);

    void startSearch();
    bool startSpeed();  // false if no search has reached the goal yet
    void abort();
    void clearMap();
    void update();  // main loop

    State state() const { return state_; }
    bool running() const { return phase_ != FINISHED; }
    bool ended() const { return state_ == DONE || state_ == STUCK; }
    bool mapReady() const { return mapReady_; }

    /// Curved turns on known-path runs (speed run and its drive home).
    void setCurves(bool on) { curves_ = on; }
    bool curves() const { return curves_; }
    uint32_t decisions() const { return decisions_; }

    /// Live telemetry: watch robot.navigator_.tel.
    struct Telemetry {
        uint8_t  state, phase, x, y, facing, lastAction;
        uint8_t  wallL, wallF, wallR;  // last sensed at a decision point
        uint8_t  frontRef;             // 1 = stop point taken from front ToF
        uint8_t  wallMode;             // 0 off, 1 both, 2 left, 3 right
        uint8_t  sideState;            // per side, bit0 L / bit1 R: side usable
                                       //  bit2 L / bit3 R: latched off (wall ended)
                                       //  bit4: side-sensor cell known from the map
        uint8_t  otherSide;            // 1 = maze found on the other side of the start
        float    remainingMm, speedCmd, wallErrMm, wallTrimDeg;
        uint32_t decisions;
        uint32_t stallStops;     // stops completed by the stall guard
        uint32_t arcs;           // curved turns taken
        int32_t  straightAhead;  // speed run: straight cells ahead
    } tel = {};

private:
    ControlLoop*     ctrl_  = nullptr;
    WallSensorArray* walls_ = nullptr;
    Encoder*         encL_  = nullptr;
    Encoder*         encR_  = nullptr;

    MazeMap   map_;
    FloodFill flood_;
    GridPose  pose_;

    State  state_   = IDLE;
    Phase  phase_   = FINISHED;
    Action pending_ = ACT_STRAIGHT;
    Turn   turn_    = TURN_NONE;

    float    cruise_        = 0.0f;
    float    runStartMm_    = 0.0f;
    float    segmentEndMm_  = 0.0f;
    float    parkEndMm_     = 0.0f;
    bool     decided_       = false;
    bool     braking_       = false;
    bool     homing_        = false;
    bool     mapReady_      = false;
    bool     otherSide_     = false;  // maze is on the other side of the start
    bool     returnKnown_   = false;  // return on explored cells only
    bool     selfParked_    = false;  // robot parked itself PARK_GAP_MM off the wall
    bool     curves_        = cfg::CURVED_TURNS_DEFAULT;
    bool     arcPending_    = false;  // a curve starts in this cell
    float    arcStartMm_    = 0.0f;   // traveled when the current curve began
    float    arcLenMm_      = 0.0f;   // curve length (encoder mm), set in init()
    bool     eventArc_      = false;  // next non-straight cell is a curve
    int      startX_        = 0;
    uint32_t decisions_     = 0;
    uint32_t settleStartMs_ = 0;
    uint32_t parkStartMs_   = 0;
    float    stallRefMm_    = 0.0f;  // stall guard: remaining at window start
    uint32_t stallSinceMs_  = 0;     // 0 = window not started

    int straightCache_ = 0;
    int cacheKey_      = -1;

    PIDController pidWall_{navcfg::WALL_PID};

    // wall-end protection, one entry per side (0 left, 1 right)
    struct SideTrack {
        float lastMm;
        bool  hasLast;
        bool  latched;
    };
    SideTrack side_[2]      = {};
    int       sideCellKey_  = -1;
    uint8_t   lastWallMode_ = 0;
    uint32_t  wallSeq_      = 0;     // side-sensor samples consumed by the wall PID
    float     lastTrim_     = 0.0f;  // wall PID output, held between samples

    float axleMm() const;
    float traveledMm() const { return axleMm() - runStartMm_; }

    void begin(State s, float cruise);
    void updateDrive();
    void updateSettle();
    void updateTurn();
    void updateArc();
    void startArc(float traveled);
    float frontArcReferenced(float traveled, float remaining);
    bool knownRun() const { return state_ == SPEED || returnKnown_; }
    bool arcsOn() const { return curves_ && knownRun(); }
    static float arcRadiusMm() { return 0.5f * cfg::CELL_TRAVEL_MM; }
    // Where the curve starts, as encoder mm before the turn cell's centre.
    static float arcStartMm() {
        return arcRadiusMm() +
               navcfg::ARC_START_ADVANCE_MM * (cfg::CELL_TRAVEL_MM / cfg::CELL_PITCH_MM);
    }
    void updatePark();
    void senseWalls();
    bool boundaryOpen(Dir side, int sensor) const;
    void moveStartToOtherSide();
    Action decide();
    void act(Action a);
    void startTurn(Turn t);
    void nextCell();
    void startHoming();
    void stopThen(Action a);
    bool stalled(float remaining);
    void finish(State s);
    float frontReferenced(float traveled, float remaining);
    float straightAheadMm();
    float wallTrimDeg(float remaining);
    bool sideUsable(int s, bool known, bool mapWall, float& mm);
    void publish(float remaining, float v);
};

#endif  // APP_NAVIGATOR_HPP
