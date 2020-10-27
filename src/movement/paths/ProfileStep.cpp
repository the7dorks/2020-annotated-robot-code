/**
 * ProfileStep.cpp
 *
 * ProfileStep is used for organizing the information parsed from motion profiles stored on the sd
 * card, calculated by the publically availible GitHub repository, TrajectoryLib by Team254 (FRC
 * Team 254, The Cheesy Poofs), found here: https://github.com/Team254/TrajectoryLib. The
 * trajectories are calculated on a computer, and stored on the sd card for the robot to use. Each
 * time step of the profile is read from the sd card, and stored in an instance of ProfileStep by
 * the Trajectory class.
 */
#include "main.h" // gives access to ProfileStep declaration and other dependencies

const std::string ProfileStep::getString() // returns the ProfileStep formatted as a std::string
                                           // without changing anything (const)
{
    return std::to_string(pos) + " " + std::to_string(vel) + " " + std::to_string(acc) + " " +
           std::to_string(jerk) + " " + std::to_string(heading) + " " + std::to_string(dt) + " " +
           std::to_string(x) + " " + std::to_string(y);
}
