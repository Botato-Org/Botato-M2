@echo off
rem Builds botato_mms.exe from the firmware's own maze code (nothing is copied).
rem mms Build Command:  cmd /c build.bat
setlocal
cd /d "%~dp0"

rem No g++ on PATH? Use the MinGW that ships with CLion (path from this repo's CMake cache).
where g++ >nul 2>nul
if errorlevel 1 set "PATH=C:\PROGRA~1\JETBRA~1\CLION2~1.3\bin\mingw\bin;%PATH%"

set APP=..\Core\App
g++ -std=c++17 -O2 -Wall -Wextra -static ^
    -I%APP%\maze -I%APP%\floodfill -I%APP%\planner ^
    -o botato_mms.exe ^
    BotatoMms.cpp API.cpp ^
    %APP%\maze\MazeMap.cpp ^
    %APP%\floodfill\FloodFill.cpp ^
    %APP%\planner\Planner.cpp
exit /b %errorlevel%
