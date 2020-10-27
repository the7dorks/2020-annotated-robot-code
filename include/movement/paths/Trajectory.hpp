/**
 * Trajectory.hpp
 *
 * This file contains the declaration of the Trajectory class.
 * The Trajectory class reads and stores motion profile information
 * from the sd card. Motion profiles stored on the sd card are
 * calculated by the publically availible GitHub repository,
 * TrajectoryLib by Team254 (FRC Team 254, The Cheesy Poofs),
 * found here: https://github.com/Team254/TrajectoryLib. The
 * trajectories are calculated on a computer, and stored on the
 * sd card for the robot to use. Each time step of the profile is
 * read from the sd card, and stored in an instance of ProfileStep
 * by the Trajectory class.
 *
 * The paths are intended to be executed by the Drivetrain class.
 */
#pragma once
#include "main.h"
class Trajectory
{
    double mkP, mkD, mkV, mkA; // control constants
    int mstepNumber; // index of current step
    double mlastErrorL, mlastErrorR; // old error values

    std::string mname; // name of the movement from the top of the file
    int mlength; // number of steps for each side to execute
    ProfileStep * mleftSteps; // steps for the left side
    ProfileStep * mrightSteps; // steps for the right side

  public:
    Trajectory(const char * ifileName, double ikP = 0.0, double ikD = 0.0, double ikV = 0.025,
               double ikA = 0.0); // constructor that specifies control constants
    ~Trajectory(); // destructor that handles dynamically allocated arrays to prevent memory issues

    std::string getName();
    int getLength();
    void reset(); // sets mstepNumber back to 0
    std::pair<ProfileStep, ProfileStep>
    getStep(int istepNumber); // get the left and right values at a certain step
    void setGains(const double ikP, const double ikD, const double ikV, const double ikA);
    bool isDone(); // checks to see if all of the steps have been executed

    std::pair<double, double>
    iterate(const double ileftDistSoFar,
            const double irightDistSoFar); // calculate the motor vales at the next step
};

namespace def
{
extern Trajectory traj_test;
extern Trajectory TestSpline;
} // namespace def