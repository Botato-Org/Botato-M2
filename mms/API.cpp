#include "API.h"

#include <cstdlib>
#include <iostream>

namespace {

std::string ask(const std::string& command) {
    std::cout << command << std::endl;       // endl flushes: mms waits for the line
    std::string reply;
    std::cin >> reply;
    return reply;
}

int  askInt(const std::string& c)  { return std::atoi(ask(c).c_str()); }
bool askBool(const std::string& c) { return ask(c) == "true"; }

} // namespace

namespace API {

int  mazeWidth()  { return askInt("mazeWidth"); }
int  mazeHeight() { return askInt("mazeHeight"); }

bool wallFront() { return askBool("wallFront"); }
bool wallRight() { return askBool("wallRight"); }
bool wallLeft()  { return askBool("wallLeft"); }

bool moveForward() { return ask("moveForward") != "crash"; }
void turnRight()   { ask("turnRight"); }
void turnLeft()    { ask("turnLeft"); }

void setWall(int x, int y, char d) {
    std::cout << "setWall " << x << " " << y << " " << d << std::endl;
}
void clearWall(int x, int y, char d) {
    std::cout << "clearWall " << x << " " << y << " " << d << std::endl;
}
void setColor(int x, int y, char c) {
    std::cout << "setColor " << x << " " << y << " " << c << std::endl;
}
void clearColor(int x, int y) {
    std::cout << "clearColor " << x << " " << y << std::endl;
}
void clearAllColor() { std::cout << "clearAllColor" << std::endl; }
void setText(int x, int y, const std::string& t) {
    std::cout << "setText " << x << " " << y << " " << t << std::endl;
}
void clearText(int x, int y) {
    std::cout << "clearText " << x << " " << y << std::endl;
}
void clearAllText() { std::cout << "clearAllText" << std::endl; }

bool wasReset() { return askBool("wasReset"); }
void ackReset() { ask("ackReset"); }

void log(const std::string& text) { std::cerr << text << std::endl; }

} // namespace API
