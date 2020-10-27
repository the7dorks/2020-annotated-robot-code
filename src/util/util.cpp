/**
 * util.cpp
 *
 * This file contains miscellaneous utility functions and classes
 * to help with the general organization of the rest of the code.
 */
#include "main.h" // gives access to util.hpp and other dependencies

/* ----------------------------------------------------------- */
/*                     ExtendedPoint Struct                    */
/* -----------------------------------------------------------  /
 * ExtendedPoint struct inherits from the built in okapi Point struct,
 * but provides additional functionality, like an orientation value
 * (theta) as well as x and y values. It also adds some vector operations that are used for path
 * following in the drivetrain class.
 */
ExtendedPoint::ExtendedPoint(QLength ix, QLength iy, QAngle itheta) : theta(itheta)
{
    x = ix;
    y = iy;
}

/* ----------------------- Subtraction ----------------------- */
ExtendedPoint ExtendedPoint::operator-(const ExtendedPoint & ivec) // overloaded subtraction
{
    return ExtendedPoint(x - ivec.x, y - ivec.y, theta - ivec.theta);
}
ExtendedPoint ExtendedPoint::sub(const ExtendedPoint & ivec) // subtraction method
{
    return *this - ivec;
}

/* ------------------------- Addition ------------------------ */
ExtendedPoint ExtendedPoint::operator+(const ExtendedPoint & ivec) // overloaded addition
{
    return ExtendedPoint(x + ivec.x, y + ivec.y, theta + ivec.theta);
}
ExtendedPoint ExtendedPoint::add(const ExtendedPoint & ivec) // addition method
{
    return *this + ivec;
}

/* ---------------------- Multiplication --------------------- */
QLength ExtendedPoint::dot(const ExtendedPoint & ivec) // dot multiply vectors
{
    return (x.convert(inch) * ivec.x.convert(inch) + y.convert(inch) * ivec.y.convert(inch)) * inch;
}
ExtendedPoint ExtendedPoint::operator*(const double iscalar) // overloaded scalar multiplication
{
    return ExtendedPoint(x * iscalar, y * iscalar, theta * iscalar);
}
ExtendedPoint ExtendedPoint::scalarMult(const double iscalar) // multiply the vectors by a scalar
{
    return *this * iscalar;
}
ExtendedPoint ExtendedPoint::operator*(const ExtendedPoint & ivec) // elementwise multiplication
{
    return ExtendedPoint((x.convert(inch) * ivec.x.convert(inch)) * inch,
                         (y.convert(inch) * ivec.y.convert(inch)) * inch,
                         (theta.convert(degree) * ivec.theta.convert(degree)) * degree);
}
ExtendedPoint ExtendedPoint::eachMult(const ExtendedPoint & ivec) // elementwise multiplication
{
    return *this * ivec;
}

/* ----------------------- Comparative ----------------------- */
bool ExtendedPoint::operator==(const ExtendedPoint & ipoint) // overloaded equivalence check
{
    return x == ipoint.x && y == ipoint.y && theta == ipoint.theta;
}

/* -------------------------- Other -------------------------- */
QLength ExtendedPoint::dist(const ExtendedPoint & ivec) // distance between points
{
    return sqrt((x - ivec.x).convert(inch) * (x - ivec.x).convert(inch) +
                (y - ivec.y).convert(inch) * (y - ivec.y).convert(inch)) *
           inch;
}
QLength ExtendedPoint::mag() // magnitude
{
    return sqrt(x.convert(inch) * x.convert(inch) + y.convert(inch) * y.convert(inch)) * inch;
}
ExtendedPoint ExtendedPoint::normalize() // creates a vector with a length of 1
{
    return ExtendedPoint((x.convert(inch) / mag().convert(inch)) * inch,
                         (y.convert(inch) / mag().convert(inch)) * inch, theta);
}
std::string ExtendedPoint::string() // returns the point in string form for testing
{
    return "{" + std::to_string(x.convert(inch)).substr(0, 3) + ", " +
           std::to_string(y.convert(inch)).substr(0, 3) + ", " +
           std::to_string(theta.convert(degree)).substr(0, 3) + "}";
}

/* ----------------------------------------------------------- */
/*                        Misc Functions                       */
/* ----------------------------------------------------------- */
void waitForImu() // blocks the execution of the code until the imu is done calibrating
{
    while (def::imu_top.is_calibrating() || def::imu_bottom.is_calibrating())
        pros::delay(100);
}

/* -------------------- OdomDebug Helpers -------------------- */
void odomSetState(OdomDebug::state_t istate) // sets the state of odometry based on display inputs
{
    CustomOdometry::setState({istate.x, istate.y, istate.theta});
}
void odomResetAll() // resets everything having to do with odometry (for "Reset" button)
{
    CustomOdometry::setState({0_ft, 0_ft, 0_deg}); // sets the robot's positinon to 0
    def::imu_top.reset(); // resets the imu
    def::imu_bottom.reset(); // resets the imu
    // resets the ecoders
    def::track_encoder_forward.reset();
    def::track_encoder_side.reset();
    waitForImu(); // waits for the imu
}

/* ---------------------- Task Functions --------------------- */
void sm_dt_task_func(void *) // state machine drivetrain task to be run independently
{
    while (true)
    {
        def::sm_dt.controlState(); // update the state from controller input
        def::sm_dt.update(); // moves the robot based on the state
        pros::delay(20);
    }
}

void sm_bc_task_func(void *) // state machine ball control task to be run independently
{
    while (true)
    {
        if (def::sm_bc.controlEnabled)
        {
            def::sm_bc.controlState(); // update the state from controller input if it is enabled
        }
        def::sm_bc.update(); // moves the robot based on the state
        pros::delay(20);
    }
}

void display_task_func(void *) // display task to be run independently
{
    while (true)
    {
        def::display.setOdomData(); // update the odometry information

        // room for any other miscellaneous debugging

        pros::delay(20);
    }
}

/* -------------------------- Macros ------------------------- */
void deploy() // deploys the robot
{
    def::sm_bc.fwShoot(); // deploys the hood
    pros::delay(250);
    def::sm_bc.fwOff();
}

/* ----------------------------------------------------------- */
/*                           Control                           */
/* ----------------------------------------------------------- */

/* ------------------------ PID Class ------------------------ /
 * PID is a feedback loop that uses the difference between the goal and the current position (error)
 * of the robot to decide how much power to give the motors. The "P" stands for "proportional", and
 * it adds power proportional to the error, so it gets slower and slower as it gets closer to the
 * goal to prevent it from driving too fast past it. The "D" stands for "derivative", because it
 * uses the derivative of the error (the speed of the robot) to apply power. The faster the robot
 * goes, the more the d term works to slow it down. The "I" stands for "integral", because it uses
 * the integral of the error (the absement of the robot) to apply power. When the robot is close to
 * the goal, sometimes the "P" and "D" terms do not apply enough power to move the robot, but when
 * the robot isn't moving (and when it is), the "I" term is acumulating, so it eventually builds up
 * enough to move the robot even closer to the goal. This implementation of PID only enables the "I"
 * term when the robot is close enough to the goal, to prevent "integral windup", which is when the
 * integral gets too big when it's too far away from the goal.
 *
 * We have a PID controller class, because we use different PID loops in many different places in
 * the code, so we wanted to be able to be able to quickly make one with constants specific to the
 * application.
 */
PID::PID(double ikP, double ikI, double ikD, double ikIRange, double isettlerError,
         double isettlerDerivative,
         QTime isettlerTime) // constructor that sets constants, and initializes variables
    : msettlerError(isettlerError), msettlerDerivative(isettlerDerivative),
      msettlerTime(isettlerTime), msettler(TimeUtilFactory::withSettledUtilParams(
                                               msettlerError, msettlerDerivative, msettlerTime)
                                               .getSettledUtil()),
      mkP(ikP), mkI(ikI), mkD(ikD), mkIRange(ikIRange), merror(0), mlastError(0), mtotalError(0),
      mderivative(0)
{
}

PID::PID(const PID & iother) // copy constructor for duplicating PID objects behind the scenes
{
    msettlerError = iother.msettlerError;
    msettlerDerivative = iother.msettlerDerivative;
    msettlerTime = iother.msettlerTime;
    msettler =
        TimeUtilFactory::withSettledUtilParams(msettlerError, msettlerDerivative, msettlerTime)
            .getSettledUtil();
    mkP = iother.mkP;
    mkI = iother.mkI;
    mkD = iother.mkD;
    mkIRange = iother.mkIRange;
    mlastError = iother.mlastError;
    mtotalError = iother.mtotalError;
    mderivative = iother.mderivative;
}

double PID::getLastError() { return mlastError; }
double PID::getTotalError() { return mtotalError; }

void PID::setGains(double ikP, double ikI, double ikD) // used only for changing constants later
{
    mkP = ikP;
    mkI = ikI;
    mkD = ikD;
}

double PID::getP() { return mkP; }
double PID::getI() { return mkI; }
double PID::getD() { return mkD; }

double PID::iterate(double ierror) // goes through one iteration of the PID loop
{
    merror = ierror;
    if (mkI != 0) // regulate integral term
    {
        if (abs(merror) < mkIRange && merror != 0) // if in range, update mtotalError
        {
            mtotalError += merror;
        }
        else
        {
            mtotalError = 0;
        }
        util::chop<double>(-50 / mkI, 50 / mkI,
                           mtotalError); // limit mtotalError to prevent integral windup
    }

    mderivative = merror - mlastError; // calculate the derivative before lastError is overwritten
    mlastError = merror; // save the current error for the next cycle

    return merror * mkP + mtotalError * mkI + mderivative * mkD;
}

bool PID::isSettled() // returns whether or not the controller is settled at the target
{
    return msettler->isSettled(merror);
}

/* ------------------------ Slew Class ------------------------ /
 * Slew rate control is a system that limits the change in speed to prevent wheel slip. If the robot
 * changes speed too fast, the wheels can slip, and make the robot's motion less fluid. When the
 * target speed changes by a lot, the slew rate controller slowly increases it's output to
 * eventually get to the target speed.
 *
 * This Slew rate controller is also intended to be used with PID, but sometimes when slew is used
 * with PID, it interferes with the settling of the PID. To prevent this, the slew rate controller
 * is only active when there are large changes in the target input value, making it only really
 * affect the beginning of the motion. For example, if the motors aren't moving, and the target
 * value suddenly jumps to 100%, the slew controller might gradually increase by increments of 5%
 * until it reaches 100%, but if the target value jumps to from0% to 20%, the slew controller might
 * not engage (actual values depend on constants "mincrement" and "mactiveDifference").
 */
Slew::Slew(double iincrement, double iactiveDifference)
    : mincrement(iincrement), mactiveDifference(iactiveDifference), mlastValue(0) // constructor
{
}

double Slew::getIncrement() { return mincrement; }
double Slew::getActiveDifference() { return mactiveDifference; }
double Slew::getLastValue() { return mlastValue; }

double Slew::iterate(double ivalue) // limits the input value to maximum changes described by
                                    // constants when run in a loop
{
    if (abs(ivalue - mlastValue) >
        mactiveDifference) // only activate if the value difference is over the threshold
    {
        if (ivalue >
            mlastValue +
                mincrement) // if the input is too big, only let it increase by a maximum amount
        {
            mlastValue = mlastValue + mincrement;
            return mlastValue;
        }
        else if (ivalue < mlastValue - mincrement) // if the input is too small, only let it
                                                   // decrease by a maximum amount
        {
            mlastValue = mlastValue - mincrement;
            return mlastValue;
        }
    }
    mlastValue = ivalue;
    return ivalue; // this only happens if nothing is wrong
}

/* ----------------------------------------------------------- */
/*                             Util                            */
/* -----------------------------------------------------------  /
 * The util namespace is used to organaize basic functions that don't
 * necessarily need to be used for robotics.
 */

/* ---------------------- Angle Wrappers ---------------------  /
 * These methods take any angle, and return an angle representing the same position in a specific
 * range. For example, wrapDeg(370) returns 10, because 370 is out of the range [0, 360).
 */
double util::wrapDeg(double iangle) // range [0, 360)
{
    iangle = fmod(iangle, 360);
    if (iangle < 0)
        iangle += 360;
    return iangle;
}
double util::wrapDeg180(double iangle) // range [-180, 180]
{
    iangle = fmod(iangle, 360);
    if (iangle < -180)
        iangle += 360;
    else if (iangle > 180)
        iangle -= 360;
    return iangle;
}
double util::wrapRad(double iangle) // range [0, 2pi)
{
    iangle = fmod(iangle, 2 * 3.14159265358979323846);
    if (iangle < 0)
        iangle += 2 * 3.14159265358979323846;
    return iangle;
}
double util::wrapRadPI(double iangle) // range [-pi, pi]
{
    iangle = fmod(iangle, 2 * 3.14159265358979323846);
    if (iangle < -3.14159265358979323846)
        iangle += 2 * 3.14159265358979323846;
    else if (iangle > 3.14159265358979323846)
        iangle -= 2 * 3.14159265358979323846;
    return iangle;
}
QAngle util::wrapQAngle(QAngle iangle) // range [0, 360) for QAngles
{
    iangle = fmod(iangle.convert(degree), 360) * degree;
    if (iangle < 0_deg)
        iangle += 360_deg;
    return iangle;
}
QAngle util::wrapQAngle180(QAngle iangle) // range [-180, 180] for QAngles
{
    iangle = fmod(iangle.convert(degree), 360) * degree;
    if (iangle < -180_deg)
        iangle += 360_deg;
    else if (iangle > 180_deg)
        iangle -= 180_deg;
    return iangle;
}
