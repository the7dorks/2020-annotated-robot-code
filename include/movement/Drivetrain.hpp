/**
 * Drivetrain.hpp
 *
 * This file contains the declaration of the Drivetrain class.
 * The Drivetrain class handles almost everthing relating to the
 * drivetrain: motor control, settings (like max speed), basic
 * movement methods (like tank or arcade), more advanced movement
 * methods (like PID to point, path following, and motion
 * profiling), and more.
 */
#pragma once // makes sure the file is only included once
#include "main.h" // gives access to dependencies from other files
class Drivetrain // creates the class for the drivetrain
{
  private:
    /* --------------------- Motor References -------------------- */
    static Motor & mmtrLeftFront;
    static Motor & mmtrRightFront;
    static Motor & mmtrRightBack;
    static Motor & mmtrLeftBack;

    /* ------------------------- Chassis ------------------------- */
    static std::shared_ptr<ChassisController>
        mchassis; // chassis object for using Pathfilder through okapi

  protected:
    /* ------------------------- Settings ------------------------ */
    static double mmaxSpeed;
    static bool menabled;

    /* -------------------- SimpleFollow Data -------------------- */
    static double mlastLookIndex; // index of the last lookahead point
    static double
        mlastPartialIndex; // fractional index of where the last lookahead point was on the segment

    /* -------------------- Odometry Accessors ------------------- */
    static OdomState getState(); // get position as OdomState
    static QLength getXPos();
    static QLength getYPos();
    static QAngle getTheta();
    static ExtendedPoint getPoint(); // get position as ExtendedPoint

    /* ------------------------- Helpers ------------------------- */
    static QAngle
    angleToPoint(const Point & itargetPoint); // calculates the field centric direction to the
                                              // itargetPoint from the robot's current position
    static std::optional<double> findIntersection(
        ExtendedPoint istart, ExtendedPoint iend,
        const double & ilookDistIn); // looks for interections between the line segment created by
                                     // the two points (istart and iend), and the circle around the
                                     // robot with radius ilookDistIn (lookahead circle)
    static ExtendedPoint
    findLookahead(SimplePath ipath,
                  const double & mlookDistIn); // looks for the intersection point between the
                                               // lookahead circle and the SimplePath, ipath

  public:
    /* --------------------- Getters/Setters --------------------- */
    static double getMaxSpeed();
    static void setMaxSpeed(const double imaxSpeed);

    static bool isEnabled();
    static void enable(); // allows movements to be startable
    static void disable(); // stops active movements

    static void
    checkNextAsync(const double & ierror,
                   std::vector<AsyncAction> &
                       iactions); // checks if the next AsyncAction should execute, and executes it
                                  // (and removes it from the list) if it should

    /* ---------------------- Basic Movement --------------------- */
    static void moveIndependant(
        double ileftFront, double irightFront, double irightBack, double ileftBack,
        const bool idesaturate = true); // moves each motor {lf, rf, rb, lb} in range [-1,1]
    static void
    moveTank(const double ileft, const double iright,
             const bool idesaturate =
                 true); // spins the left side and right side motors at certian speeds [-1,1]
    static void moveArcade(
        const double iforward, const double istrafe, const double iturn,
        const bool idesaturate = true); // moves the robot with arcade-style inputs (range[-1,1])

    /* ------------------ Intermediate Movement ------------------ */
    static void moveInDirection(
        QAngle idirection, const bool ifieldCentric = false, double imagnitude = 1,
        double itheta = 0,
        const bool idesaturate = true); // moves the robot with a certain speed in a certain
                                        // direction, while turning a certain amount

    /* ------------------ Move to Point Methods ------------------ */
    static void strafeToPoint(
        ExtendedPoint iPoint, std::vector<AsyncAction> iactions = {},
        PID imagnitudePID = PID(0.4, 0.005, 2.6, 0.5, 0.25, 0.05, 10_ms),
        PID iturnPID = PID(0.028, 0.0, 0.08, 0.0, 1.5, 0.1, 10_ms),
        Slew imagnitudeSlew = Slew(1000, 1000),
        Slew iturnSlew =
            Slew(1000, 1000)); // drives in a stright line to the point while turning using set
                               // PID/Slew gains, and executing the AsyncActions at the right times

    static void straightToPoint(
        ExtendedPoint itarget, std::vector<AsyncAction> iactions = {}, QLength inoTurnRange = 3_in,
        double iturnWeight = 1.7, PID imagnitudePID = PID(0.4, 0.005, 2.6, 0.5, 0.25, 0.05, 10_ms),
        PID iturnPID = PID(0.028, 0.0, 0.08, 0.0, 1.5, 0.1, 10_ms),
        Slew imagnitudeSlew = Slew(1000, 1000),
        Slew iturnSlew = Slew(1000,
                              1000)); // drives to the point without strafing using set PID/Slew
                                      // gains, and executing the AsyncActions at the right times

    static void arcStraightToPoint(
        ExtendedPoint itarget, std::vector<AsyncAction> iactions = {}, double iweightModifier = 10,
        QLength inoTurnRange = 3_in,
        PID imagnitudePID = PID(0.4, 0.005, 2.6, 0.5, 0.25, 0.05, 10_ms),
        PID iturnPID = PID(0.028, 0.0, 0.08, 0.0, 1.5, 0.1, 10_ms),
        Slew imagnitudeSlew = Slew(1000, 1000),
        Slew iturnSlew = Slew(
            1000,
            1000)); // drive in an "arc" (doesn't follow a path, just approximates an arc) using set
                    // PID/Slew gains, and executing the AsyncActions at the right times

    /* ------------------ Path Following Methods ----------------- */
    static void simpleFollow(
        SimplePath ipath, QLength ilookDist = 6_in, std::vector<AsyncAction> iactions = {},
        PID imagnitudePID = PID(0.4, 0.005, 2.6, 0.5, 0.25, 0.05, 10_ms),
        PID iturnPID = PID(0.028, 0.0, 0.08, 0.0, 1.5, 0.1, 10_ms),
        Slew imagnitudeSlew = Slew(1000, 1000),
        Slew iturnSlew = Slew(1000,
                              1000)); // follows the path, ipath using set lookahead distance
                                      // (ilookDist) and PID/Slew gains while executing the
                                      // AsyncActions at the right times (only on the last segment)

    /* --------------------- Motion Profiling -------------------- */
    static std::shared_ptr<AsyncMotionProfileController>
        mprofiler; // okapi motion profile controller
    static void generatePathToPoint(
        PathfinderPoint ipoint,
        const std::string & iname); // use Pathfinder through okapi to make a motion profile
    static void followPathfinder(const std::string & iname, bool ibackwards = false,
                                 bool imirrored = false); // follow Pathfinder path through okapi
    static void followTraj(Trajectory & itraj); // follow trajectory loaded from sd card
};

namespace def
{
extern Drivetrain drivetrain; // declares the drivetrain object as extern, to make sure it only gets
                              // constructed once
}