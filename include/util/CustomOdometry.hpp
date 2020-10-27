/**
 * CustomOdometry.hpp
 *
 * This file contains the declaration of the CustomOdometry class.
 * CustomOdometry is resposible for doing all the math and storing
 * information about the robot's position and orientation. Everthing
 * is static, because there doesn't need to be more than one position
 * calculation.
 */
#pragma once         // makes sure the file is only included once
#include "main.h"    // give access to dependencies from other files
class CustomOdometry // class that organizes the sensors, calculations, and state variables for odometry
{

    /* ------------------------ Constants ------------------------ */
    static const double & moffFIn; // offset of forward tracking wheel in inches
    static const double & moffSIn; // offset of side tracking wheel in inches
    static const double & mcircIn; // tracking wheel circumference in inches

    /* -------------------- Sensor References -------------------- */
    static ADIEncoder & meF;  // left tracking wheel encoder
    static ADIEncoder & meS;  // right tracking wheel encoder
    static pros::Imu & mimu1; // inertial sensors
    static pros::Imu & mimu2;

    /* ------------------------ Variables ------------------------ */
    static OdomState mstate; // position of the robot
    static bool menabled;    // whether or not the loop is allowed to run

    /* ------------------------- Methods ------------------------- */
    static std::valarray<double> getSensorVals(); // returns new sensor values
    friend void odomTaskFunc(void *);             // task to be run all the time.

  public:
    static OdomState getState();                    // returns the current state of the robot
    static QLength getX();                          // returns the x value of the state
    static QLength getY();                          // returns the y value of the state
    static QAngle getTheta();                       // returns the theta value of the state
    static void setState(const OdomState & istate); // sets the state of the robot

    static void enable();  // allows the odometry thread to be started (but does not start it)
    static void disable(); // stops the odometry thread from running, prevents it from starting

    static OdomState mathStep(std::valarray<double> ivalsDiff); // does one iteration of odometry math, given sensor changes
};

namespace def
{
extern CustomOdometry customOdom; // declares the customOdom object as extern, to make sure it only gets constructed once
}

void odomTaskFunc(void *); // friend function to CustomOdometry to be run as a separate thread
