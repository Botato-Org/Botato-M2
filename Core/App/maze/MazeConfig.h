/**
 * @file  MazeConfig.h
 * @brief Logical maze: size, start corner and goal box.
 *
 * x = column (0 = left), y = row (0 = bottom, the start row). NORTH = +y; the robot starts
 * facing NORTH. A wrong start corner is detected and corrected at runtime (DECISIONS #30).
 */
#ifndef APP_MAZECONFIG_H
#define APP_MAZECONFIG_H

namespace maze {

constexpr int WIDTH   = 16;
constexpr int HEIGHT  = 16;
constexpr int MAX_DIM = 16;  // static array bound, keep >= WIDTH/HEIGHT

// Facing out of the start cell: maze on the RIGHT -> 0, on the LEFT -> WIDTH - 1.
constexpr int START_X = WIDTH - 1;  // ROBOFEST 2026 prelim: maze on the left
constexpr int START_Y = 0;

// Inclusive goal box: entering any cell inside counts as reached.
constexpr int GOAL_X_MIN = 7;
constexpr int GOAL_X_MAX = 8;
constexpr int GOAL_Y_MIN = 7;
constexpr int GOAL_Y_MAX = 8;

static_assert(WIDTH <= MAX_DIM && HEIGHT <= MAX_DIM, "maze larger than MAX_DIM");
static_assert(START_X == 0 || START_X == WIDTH - 1, "start must be in a corner column");
static_assert(GOAL_X_MIN <= GOAL_X_MAX && GOAL_Y_MIN <= GOAL_Y_MAX, "bad goal box");

}  // namespace maze

#endif  // APP_MAZECONFIG_H
