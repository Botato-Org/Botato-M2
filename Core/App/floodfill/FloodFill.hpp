/**
 * @file  FloodFill.hpp
 * @brief BFS step distance to a target over the known map. No hardware.
 */
#ifndef APP_FLOODFILL_HPP
#define APP_FLOODFILL_HPP

#include <cstdint>
#include "MazeMap.hpp"

/**
 * knownOnly = false (search, return): unknown walls are assumed open.
 * knownOnly = true (speed run): only visited cells are traversable.
 */
class FloodFill {
public:
    static constexpr uint16_t INF = 0xFFFF;

    void toGoal(const MazeMap& map, bool knownOnly);
    void toCell(const MazeMap& map, int tx, int ty, bool knownOnly);

    uint16_t distance(int x, int y) const { return distance_[x][y]; }

    // Open neighbour with the lowest distance. Ties go to `prefer` (the
    // current heading) so the robot does not turn when it doesn't need to.
    bool nextDir(const MazeMap& map, int x, int y, Dir prefer, Dir& out) const;

private:
    uint16_t distance_[maze::MAX_DIM][maze::MAX_DIM] = {};
    void flood(const MazeMap& map, const int (*seeds)[2], int count, bool knownOnly);
};

#endif  // APP_FLOODFILL_HPP
