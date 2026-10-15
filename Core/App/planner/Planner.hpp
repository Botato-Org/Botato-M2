/**
 * @file  Planner.hpp
 * @brief Grid pose and absolute-to-relative turn translation. No hardware.
 */
#ifndef APP_PLANNER_HPP
#define APP_PLANNER_HPP

#include "MazeMap.hpp"

/**
 * turnFor() is purely geometric (Dir is clockwise: N=0, E=1, S=2, W=3). turnDegrees() is the
 * only place a Turn maps onto the gyro's anticlockwise-positive convention (DECISIONS #21).
 */
enum Turn : uint8_t { TURN_NONE, TURN_LEFT, TURN_RIGHT, TURN_AROUND };

struct GridPose {
    int x      = 0;
    int y      = 0;
    Dir facing = NORTH;
};

class Planner {
public:
    static Turn turnFor(Dir facing, Dir target);
    static float turnDegrees(Turn t);  // +90 left, -90 right, +180 around
    static void stepForward(GridPose& p);
    static void applyTurn(GridPose& p, Turn t);
};

#endif  // APP_PLANNER_HPP
