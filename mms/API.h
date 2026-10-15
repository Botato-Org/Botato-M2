#ifndef MMS_API_H
#define MMS_API_H

#include <string>

/*
 * API — the mms (mackorone Micromouse Simulator) text protocol.
 * Commands go out on stdout, replies come back on stdin, log text goes to
 * stderr (shown in the simulator's run output pane).
 */
namespace API {

int  mazeWidth();
int  mazeHeight();

bool wallFront();
bool wallRight();
bool wallLeft();

bool moveForward();          // false = crashed into a wall
void turnRight();
void turnLeft();

void setWall(int x, int y, char direction);    // 'n' 'e' 's' 'w'
void clearWall(int x, int y, char direction);
void setColor(int x, int y, char color);
void clearColor(int x, int y);
void clearAllColor();
void setText(int x, int y, const std::string& text);
void clearText(int x, int y);
void clearAllText();

bool wasReset();
void ackReset();

void log(const std::string& text);

} // namespace API

#endif // MMS_API_H
