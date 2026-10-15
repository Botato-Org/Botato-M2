# BOTATO in mms

Runs the firmware's flood fill in [mms](https://github.com/mackorone/mms) (the Micromouse Simulator).

Nothing is copied. The build compiles these files straight from the firmware tree:

| File | What it gives |
|---|---|
| `Core/App/floodfill/FloodFill.cpp/.hpp` | the flood fill |
| `Core/App/maze/MazeMap.cpp/.hpp`, `MazeConfig.h` | wall map, maze size, start, goal box |
| `Core/App/planner/Planner.cpp/.hpp` | pose and turn logic |

Change any of them, press **Build** in mms, and the simulator runs the new code.

## Set up in mms (once)

Mouse → **+** (new mouse config):

| Field | Value |
|---|---|
| Name | `BOTATO` |
| Directory | `C:\New volume\Botato\MicroMouse\Botato-Micromouse-1.0\mms` |
| Build Command | `cmd /c build.bat` |
| Run Command | `cmd /c botato_mms.exe --mirror` |

Load any 16×16 classic maze, then **Build** and **Run**.

`build.bat` uses `g++` from your PATH. If there isn't one, it falls back to the MinGW that comes with CLion.

## Run options (add them to the Run Command)

| Option | Effect |
|---|---|
| `--mirror` | The robot sees the mms maze mirrored, so the maze is on its **left**. This is the ROBOFEST 2026 layout and matches `START_X = WIDTH-1`. |
| *(no `--mirror`)* | The maze is on the robot's right, as mms normally has it. The firmware's start-corner detection has to work that out, which tests that code too. |
| `--speed-runs N` | Number of speed runs after the search (default 1). |
| `--no-text` | Don't draw the flood distances (runs faster). |

The run order is the same as on the robot. First a search to the goal, then the return home, still exploring. Then speed runs over visited cells only, each one driving itself home. The **Reset** button in mms works like lifting the robot back to the start: the map is kept. If the goal was already reached, the next run is a speed run. If not, a fresh search starts.

On the mms map: dark blue = search, cyan = return while exploring, dark green = speed run, green = return over known cells. The numbers are FloodFill distances. The log pane shows cells, turns and decisions for each leg, plus the length of the known path.

## What is *not* shared with the firmware

`Navigator.cpp` drives motors, encoders and ToF sensors, so it can't run in mms. `BotatoMms.cpp` has a copy of its cell-level decision logic (`senseWalls`, `decide`, `act`, start-corner detection, homing). Those copies are the functions marked `>>> twin of Navigator::...`.

- Changing **FloodFill / MazeMap / MazeConfig / Planner**: nothing else to do.
- Changing the **search policy in Navigator.cpp**: make the same change in those marked functions.
- Changing **FloodFill's public functions** (`toGoal`, `toCell`, `nextDir`, `distance`): update the calls in `BotatoMms.cpp`.
- Adding a **new source file** that FloodFill depends on: add it to `build.bat` / `build.sh`.
