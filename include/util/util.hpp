/**
 * util.hpp
 *
 * This file contains miscellaneous utility functions and classes
 * to help with the general organization of the rest of the code.
 */
#pragma once      // makes sure the file is only included once
#include "main.h" // gives access to dependancies from other files

/* ----------------------------------------------------------- */
/*                     ExtendedPoint Struct                    */
/* -----------------------------------------------------------  /
 * ExtendedPoint struct inherits from the built in okapi Point struct,
 * but provides additional functionality, like an orientation value
 * (theta) as well as x and y values. It also adds some vector operations.
 */
struct ExtendedPoint : Point
{
    ExtendedPoint(QLength ix, QLength iy, QAngle itheta);

    QAngle theta{0_deg}; // stores the orientation at the point, with a default of 0 degrees

    /* ----------------------- Subtraction ----------------------- */
    ExtendedPoint operator-(const ExtendedPoint & ivec);
    ExtendedPoint sub(const ExtendedPoint & ivec);

    /* ------------------------- Addition ------------------------ */
    ExtendedPoint operator+(const ExtendedPoint & ivec);
    ExtendedPoint add(const ExtendedPoint & ivec);

    /* ---------------------- Multipliation ---------------------- */
    QLength dot(const ExtendedPoint & ivec); // dot multiply the vectors
    ExtendedPoint operator*(const double iscalar);
    ExtendedPoint scalarMult(const double iscalar);      // multiply the vectors by a scalar
    ExtendedPoint operator*(const ExtendedPoint & ivec); // elementwise multiplication
    ExtendedPoint eachMult(const ExtendedPoint & ivec);

    /* ----------------------- Comparative ----------------------- */
    bool operator==(const ExtendedPoint & ipoint); // checks to see of the points are the same

    /* -------------------------- Other -------------------------- */
    QLength dist(const ExtendedPoint & ivec); // distance between points
    QLength mag();                            // magnitude
    ExtendedPoint normalize();                // creates a vector with a length of 1
    std::string string();
};

/* ----------------------------------------------------------- */
/*                        Misc Functions                       */
/* ----------------------------------------------------------- */
void waitForImu(); // blocks execution of the code until the imu is done calibrating

/* -------------------- OdomDebug Helpers -------------------- */
void odomSetState(OdomDebug::state_t istate); // sets the state of odometry based on display inputs
void odomResetAll();                          // resets everything having to do with odometry (for "Reset" button)

/* ---------------------- Task Functions --------------------- */
void sm_dt_task_func(void *); // state machine drivetrain task to be run independently
void sm_bc_task_func(void *); // state machine ball control task to be run independently

void display_task_func(void *); // display task to be run independently

/* -------------------------- Macros ------------------------- */
void deploy(); // deploys the robot

/* ----------------------------------------------------------- */
/*                           Control                           */
/* ----------------------------------------------------------- */

/* ------------------------ PID Class ------------------------ /
 * PID is a feedback loop that uses the difference between the goal
 * and the current position (error) of the robot to decide how much
 * power to give the motors. The "P" stands for "proportional", and
 * it adds power proportional to the error, so it gets slower and
 * slower as it gets closer to the goal to prevent it from driving
 * too fast passed it. The "D" stands for "derivative", because it
 * uses the derivative of the error (the speed of the robot) to apply
 * power. If the robot is moving too fast, the "D" term will slow
 * down, and if it is moving too slow, the "D" term will speed up.
 * The "I" stands for "integral", because it uses the integral of the
 * error (the absement of the robot) to apply power. When the robot
 * is close to the goal, sometimes the "P" and "D" terms do not
 * apply enough power to move the robot, but when the robot isn't
 * moving, the "I" term is acumulating, so it eventually builds up
 * enough to get the robot even closer to the goal. This implementation
 * of PID only enables the "I" term when the robot is close enough
 * to the goal, to prevent "integral windup", which is when the
 * integral gets too big when it's too far away from the goal.
 *
 * We have a PID controller class, because we use different PID loops
 * in many different places in the code, so we wanted to be able to
 * be able to quickly make one with constants specific to the application.
 */
class PID
{
    double msettlerError, msettlerDerivative; // target error and derivative for the settler
    QTime msettlerTime;                       // target time for the settler
    std::unique_ptr<SettledUtil> msettler;    // okapi settler that is used to determine if the PID should stop, based on error, derivative, and time

    double mkP, mkI, mkD, mkIRange; // constants

    double merror, mlastError, mtotalError;
    double mderivative; // used for storing derivative before lastError is overwritten

  public:
    PID(double ikP, double ikI, double ikD, double ikIRange, double isettlerError, double isettlerDerivative, QTime isettlerTime); // constructor

    PID(const PID & iother); // copy constructor

    double getLastError();
    double getTotalError();

    void setGains(double ikP, double ikI, double ikD);

    double getP();
    double getI();
    double getD();

    double iterate(double ierror); // goes through one iteration of the PID loop

    bool isSettled(); // returns whether or not the controller is settled at the target
};

/* ------------------------ Slew Class ------------------------ /
 * Slew rate control is a system that limits the change in speed to
 * prevent wheel slip. If the robot changes speed to fast, the wheels
 * can slip, and make the robot's motion less fluid. When the target
 * speed changes by a lot, the slew rate controller slowly increases
 * it's output to eventually get to the target speed.
 *
 * This Slew rate controller is also intended to be used with PID, but
 * sometimes when slew is used with PID, it interferes with the settling
 * of the PID. To prevent this, the slew rate controller is only active
 * when there are large changes in the target input value, making it only
 * really affect the beginning of the motion. For example, if the motors
 * aren't moving, and the target value suddenly jumps to 100%, the slew
 * controller might gradually increase by increments of 5% until it
 * reaches 100%, but if the target value jumps to from0% to 20%, the
 * slew controller might not engage (actual values depend on constants
 * "mincrement" and "mactiveDifference").
 */
class Slew
{
    double mincrement;        // amount to change between each iteration
    double mactiveDifference; // threshold to activate slew
    double mlastValue;        // previous value

  public:
    Slew(double iincrement, double iactiveDifference); // constructor

    double getIncrement();
    double getActiveDifference();
    double getLastValue();

    double iterate(double ivalue); // limits the input value to maximum changes described by constants when run in a loop
};

/* ----------------------------------------------------------- */
/*                             Util                            */
/* -----------------------------------------------------------  /
 * The util namespace is used to organaize basic functions that don't
 * necessarily need to be used for robotics.
 */
namespace util
{

/* ----------------------- DEMA Filter -----------------------  /
 * DEMA is short for Double Exponential Moving Average. It is a method
 * is a type of filter that smooths data and gives more weight to
 * more recent values.
 *
 * A Simple Moving Average (SMA) takes the mean of a certain number
 * of values over a specified period of time. An Exponential Moving
 * Average (EMA) is similar, but it gives more weight to newer values,
 * so it more closly tracks the actual value. A DEMA is the EMA of
 * an EMA. More specifically, it is calculated by 2EMA - EMA(EMA),
 * and it gives even more weight to newer values.
 *
 * The DEMAFilter class was originally added as an easy way to improve
 * the quality of angle measurements from the inertial sensor. It was
 * needed because the odometry loop updates at 100hz, and the inertial
 * sensors used to only update at 50hz. The DEMA filter did improve
 * the position calculation a small amount, but now the inertial
 * sensor can update at 100hz (maybe more; it's unclear), and the
 * filter is no longer useful.
 */
template <int N> // the DEMA filter can be set to use the previous N values, changing how significant newer values are
class DEMAFilter
{
    const double mk;              // weighting constant
    double mlastEMA, mlastEMAEMA; // previous EMA values

    double EMACalc(double & inext, double & iold) { return (inext - iold) * mk + iold; } // EMA calculation

  public:
    DEMAFilter(std::array<double, 2 * N - 1> ifirstVals) : mk(2.0 / (N + 1)) // to start filtering values, the DEMA filter needs to have pre-filtered values. The constructor calculates the "last" values of the EMA and EMA(EMA)
    {
        for (int i = 0; i < N; i++) // calc sum of the first N numbers
        {
            mlastEMA += ifirstVals[i];
        }
        mlastEMA /= N; // store the SMA (mean) of the first N numbers

        mlastEMAEMA = mlastEMA;
        for (int i = 0; i < N - 1; i++)
        {
            mlastEMA = EMACalc(ifirstVals[i + N], mlastEMA); // put the next values from the input through EMA filter
            mlastEMAEMA += mlastEMA;                         // store sum of these values
        }
        mlastEMAEMA /= N; // store the SMA (mean) of the first N values from the EMA
    }

    double filter(double iinput) // filters the input value by doing DEMA calculation
    {
        double EMA = EMACalc(iinput, mlastEMA);    // first EMA
        double EMAEMA = EMACalc(EMA, mlastEMAEMA); // EMA of first EMA (EMA(EMA))
        mlastEMA = EMA;                            // store previous EMA
        mlastEMAEMA = EMAEMA;                      // store previous EMA(EMA)

        return 2 * EMA - EMAEMA; // 2EMA - EMA(EMA)
    }
};

/* ---------------------- Angle Wrappers ---------------------  /
 * All of these functions take an angle as an input, and return an
 * angle fitting into a certain range. For example, wrapDeg(370) would
 * return 10, and wrapDeg180(200) would return -160
 */
double wrapDeg(double iangle);       // [0, 360)
double wrapDeg180(double iangle);    // [-180, 180)
double wrapRad(double iangle);       // [0, 2pi)
double wrapRadPI(double iangle);     // [-pi, pi)
QAngle wrapQAngle(QAngle iangle);    // [0_deg, 360_deg)
QAngle wrapQAngle180(QAngle iangle); // [-180_deg, 180_deg)

/* ------------------------- Find Max ------------------------  /
 * these functions all find the maximum value of a few different types
 * of inputs. They use templates so they can be used on different types,
 * and on arrays of different lengths.
 */
template <class T, std::size_t N>
T findMax(const std::array<T, N> && iarray) // returns the max value in iarray
{
    T largest = iarray.at(0);    // gives largest a value to compare with
    for (const T & val : iarray) // loops through all values
        if (val > largest)
            largest = val; // stores the largest value
    return largest;
}
template <class T, std::size_t N>
T findMax(const std::array<T, N> & iarray) // returns the max value in iarray
{
    T largest = iarray.at(0);    // gives largest a value to compare with
    for (const T & val : iarray) // loops through all values
        if (val > largest)
            largest = val; // stores the largest value
    return largest;
}
template <class T, std::size_t N>
T findAbsMax(const std::array<T, N> && iarray) // returns the max absolute value in iarray
{
    T largest = iarray.at(0);    // gives largest a value to compare with
    for (const T & val : iarray) // loops through all values
        if (abs(val) > largest)
            largest = abs(val); // stores the largest value
    return largest;
}
template <class T, std::size_t N>
T findAbsMax(const std::array<T, N> & iarray) // returns the max absolute value in iarray
{
    T largest = iarray.at(0);    // gives largest a value to compare with
    for (const T & val : iarray) // loops through all values
        if (abs(val) > largest)
            largest = abs(val); // stores the largest value
    return largest;
}

/* ------------------------- Fitters -------------------------  /
 * These functions modfify the input to fit in a specified range
 */
template <std::size_t N>
std::array<double, N> scaleToFit(double imagnitude, std::array<double, N> && iarray) // scales all elements in iarray to fit within [-imagnitude, imagnitude]
{
    double largest = findAbsMax<double, N>(iarray);
    if (largest > imagnitude) // if anything is out of range
    {
        largest = std::abs(largest);
        for (double & val : iarray) // scales everything down to fit in the range
            val = val / largest * imagnitude;
    }
    return iarray;
}
template <std::size_t N>
void scaleToFit(double imagnitude, std::array<double, N> & iarray) // scales all elements in iarray to fit within [-imagnitude, imagnitude]
{
    double largest = findAbsMax<double, N>(iarray);
    if (largest > imagnitude) // if anything is out of range
    {
        largest = std::abs(largest);
        for (double & val : iarray) // scales everything down to fit in the range
            val = val / largest * imagnitude;
    }
}

template <class T, std::size_t N>
void chop(T imin, T imax, std::array<T, N> & iarray) // if any values in iarray are out of range, they are set to the limit
{
    for (double & val : iarray)
    {
        if (val > imax)
            val = imax;
        else if (val < imin)
            val = imin;
    }
}
template <class T>
void chop(T imin, T imax, T & inum) // if the value is out of range, it is set to the limit
{
    if (inum > imax)
        inum = imax;
    else if (inum < imin)
        inum = imin;
}
} // namespace util