#!/bin/sh
# Builds botato_mms from the firmware's own maze code (nothing is copied).
# mms Build Command:  sh build.sh
set -e
cd "$(dirname "$0")"
APP=../Core/App
g++ -std=c++17 -O2 -Wall -Wextra \
    -I$APP/maze -I$APP/floodfill -I$APP/planner \
    -o botato_mms \
    BotatoMms.cpp API.cpp \
    $APP/maze/MazeMap.cpp \
    $APP/floodfill/FloodFill.cpp \
    $APP/planner/Planner.cpp
