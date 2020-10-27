# robot-code
This repository contains all of the code used by 333A's robot in the 2020-2021 VRC Annotated Programming Skills Challenge.
## How to Use This Code
### For Reading
The relevant code in this repository is organized by two types of files: `#include`ed c++ header files (`.hpp`) in the "include" folder, and their corresponding c++ source files (`.cpp`) in the "src" folder. All of the files not in the "include" or "src" folders are only used for compiling/downloading the code to the robot.
#### `robot-code.pdf`
It is possible to read the code directly from GitHub, but for the best experience, it is recomended that you read robot-code.pdf, which is formatted for readibility and does not include files provided by pros.
### For Use on a Robot
Dowload the files, then run `prosv5 upload all` with a v5 brain connected (you must have pros cli installed).
## File Map
Crossed out files and directories were provided by pros.
#### ~~bin~~
#### ~~firmware~~
#### include
* ~~display~~
* gui
  * `DisplayControl.hpp`
  * `odomDebug.hpp`
* movement
  * paths
    * `ProfileStep.hpp`
    * `SimplePath.hpp`
    * `Trajectory.hpp`
  * `AsyncAction.hpp`
  * `Drivetrain.hpp`
* ~~okapi~~
* ~~pros~~
* stateMachines
  * `BallControlStateMachine.hpp`
  * `DrivetrainStateMachine.hpp`
  * `VStateMachine.hpp`
* util
  * `CustomOdometry.hpp`
  * `definitions.hpp`
  * `util.hpp`
* ~~api.h~~
* `Auton.hpp`
* `main.h`
#### src
* gui
  * `DisplayControl.cpp`
  * `odomDebug.cpp`
* movement
  * paths
    * `ProfileStep.cpp`
    * `SimplePath.cpp`
    * `Trajectory.cpp` 
  * `Drivetrain.cpp`
* stateMachines
  * `BallControlStateMachine.cpp`
  * `DrivetrainStateMachine.cpp`
* util
  * `CustomOdometry.cpp`
  * `util.cpp`
* `Auton.cpp`
* `definitions.cpp`
* `main.cpp`
#### ~~common.mk~~
#### ~~compile_commands~~
#### ~~Makefile~~
#### ~~project~~
