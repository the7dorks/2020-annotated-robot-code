/**
 * CustomOdometry.cpp
 *
 * This file contains the declaration of the CustomOdometry class.
 * CustomOdometry is resposible for doing all the math and storing
 * information about the robot's position and orientation. Everthing
 * is static, because there doesn't need to be more than one position
 * calculation.
 */
#include "main.h" // gives access to CustomOdometry and other dependencies

/* ----------------------------------------------------------- */
/*                     Private Information                     */
/* ----------------------------------------------------------- */

/* ------------------------ Constants ------------------------ */
const double & CustomOdometry::moffFIn =
    def::TRACK_FORWARD_OFFSET.convert(inch); // offset of forward tracking wheel in inches
const double & CustomOdometry::moffSIn =
    def::TRACK_SIDE_OFFSET.convert(inch); // offset of side tracking wheel in inches
const double & CustomOdometry::mcircIn =
    def::TRACK_WHEEL_CIRCUMFERENCE.convert(inch); // tracking wheel circumference in inches

/* -------------------- Sensor References -------------------- */
ADIEncoder & CustomOdometry::meF = def::track_encoder_forward; // left tracking wheel encoder
ADIEncoder & CustomOdometry::meS = def::track_encoder_side; // right tracking wheel encoder
pros::Imu & CustomOdometry::mimu1 = def::imu_bottom; // inertial sensors
pros::Imu & CustomOdometry::mimu2 = def::imu_top;

/* --------------------- Starting Values --------------------- */
OdomState CustomOdometry::mstate = {0_in, 0_in, 0_rad}; // position of the robot
bool CustomOdometry::menabled = true; // whether or not the loop is allowed to run

/* ------------------------- Methods ------------------------- */
std::valarray<double> CustomOdometry::getSensorVals() // returns new sensor values
{
    return {meF.get(), meS.get(),
            ((isinf(mimu1.get_rotation()) ? 0 : mimu1.get_rotation()) +
             (isinf(mimu2.get_rotation()) ? 0 : mimu2.get_rotation())) *
                M_PI / 180 / 2};
}

/* ----------------------------------------------------------- */
/*                      Public Information                     */
/* ----------------------------------------------------------- */
OdomState CustomOdometry::getState() { return mstate; } // returns the current state of the robot
QLength CustomOdometry::getX() { return mstate.x; } // returns the x value of the state
QLength CustomOdometry::getY() { return mstate.y; } // returns the y value of the state
QAngle CustomOdometry::getTheta() { return mstate.theta; } // returns the theta value of the state
void CustomOdometry::setState(const OdomState & istate) // sets the state of the robot
{
    mstate = istate;
}

void CustomOdometry::enable() // allows the odometry thread to be started (but does not start it)
{
    menabled = true;
}
void CustomOdometry::disable() // stops the odometry thread from running, prevents it from starting
{
    menabled = false;
}

OdomState
CustomOdometry::mathStep(const std::valarray<double>
                             ivalsDiff) // does one iteration of odometry math, given sensor changes
{
    const double df =
        ivalsDiff[0] * mcircIn / 360; // stores the change of all tracking wheels in inches
    const double ds = ivalsDiff[1] * mcircIn / 360;

    double vectorLx, vectorLy; // declares local offset x and y
    if (ivalsDiff[2]) // if the robot turned
    {
        vectorLx = 2 * sin(ivalsDiff[2] / 2) *
                   (ds / ivalsDiff[2] + moffSIn); // sideways translation based on arc
        vectorLy = 2 * sin(ivalsDiff[2] / 2) *
                   (df / ivalsDiff[2] + moffFIn); // forward translation based on arc
    }
    else
    {
        vectorLx = ds; // sideways translation (without turning)
        vectorLy = df; // forward translation (without turning)
    }

    if (isnan(vectorLy)) // makes sure the local offsets exist
        vectorLy = 0;
    if (isnan(vectorLx))
        vectorLx = 0;

    double avgT =
        mstate.theta.convert(radian) + ivalsDiff[2] / 2; // calculates the direction the robot moved

    double polarR = hypot(vectorLx, vectorLy); // calculates polar cordinate, r
    double polarT =
        atan2(vectorLy, vectorLx) - avgT; // calculates polar cordinate, theta, and rotates

    double dx = sin(polarT) * polarR; // converts new polar coordinates back to cartesian
    double dy = cos(polarT) * polarR;

    if (isnan(dx)) // makes sure the cartesian coordinates exist
        dx = 0;
    if (isnan(dx))
        dy = 0;

    return {dx * inch, dy * inch}; // return the change in position
}

/* ----------------------------------------------------------- */
/*                        Friend Method                        */
/* ----------------------------------------------------------- */
void odomTaskFunc(void *) // friend function to CustomOdometry to be run as a separate thread
{
    std::valarray<double> lastVals{0, 0, 0},
        newVals{0, 0, 0}; // arrays used for storing sensor values
    OdomState newState; // used to store the change in state
    waitForImu(); // wait for the inertial sensors to calibrate
    CustomOdometry::mstate = def::SET_ODOM_START; // set the starting position to the origin

    while (CustomOdometry::menabled)
    {
        newVals = CustomOdometry::getSensorVals(); // provides new sensor values and saves them
        newState = CustomOdometry::mathStep(
            newVals -
            lastVals); // runs odometry math on sensor value change to calulate change in state
        lastVals = newVals; // stores sensor values for the next iteration

        // updates state based on change
        CustomOdometry::mstate.x += newState.x;
        CustomOdometry::mstate.y += newState.y;
        CustomOdometry::mstate.theta = newVals[2] * radian;

        pros::delay(10); // run odometry at 100hz (every 10 ms)
    }
}
