/**
 * ProfileStep.hpp
 *
 * ProfileStep is used for organizing the information parsed from
 * motion profiles stored on the sd card, calculated by the
 * publically availible GitHub repository, TrajectoryLib by
 * Team254 (FRC Team 254, The Cheesy Poofs), found here:
 * https://github.com/Team254/TrajectoryLib. The trajectories
 * are calculated on a computer, and stored on the sd card for
 * the robot to use. Each time step of the profile is read from
 * the sd card, and stored in an instance of ProfileStep by the
 * Trajectory class.
 */
#pragma once
#include "main.h"
struct ProfileStep
{
    float pos{0.000};
    float vel{0.000};
    float acc{0.000};
    float jerk{0.000};
    float heading{0.000};
    float dt{0.000};
    float x{0.000};
    float y{0.000};

    const std::string getString(); // return the contents of the ProfileStep for testing purposes
};