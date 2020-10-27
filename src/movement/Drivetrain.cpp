/**
 * Drivetrain.cpp
 *
 * This file contains the definitions of the Drivetrain class.
 * The Drivetrain class handles almost everthing relating to the
 * drivetrain: motor control, settings (like max speed), basic
 * movement methods (like tank or arcade), more advanced movement
 * methods (like PID to point, path following, and motion
 * profiling), and more.
 */
#include "main.h" // gives access to Drivetrain and other dependancies

/* ----------------------------------------------------------- */
/*                     Private Information                     */
/* ----------------------------------------------------------- */

/* --------------------- Motor References -------------------- */
Motor & Drivetrain::mmtrLeftFront = def::mtr_dt_left_front;
Motor & Drivetrain::mmtrRightFront = def::mtr_dt_right_front;
Motor & Drivetrain::mmtrRightBack = def::mtr_dt_right_back;
Motor & Drivetrain::mmtrLeftBack = def::mtr_dt_left_back;

/* ---------------------- Okapi Chassis ---------------------- */
std::shared_ptr<ChassisController> Drivetrain::mchassis =
    ChassisControllerBuilder()
        .withMotors({Drivetrain::mmtrLeftFront, Drivetrain::mmtrLeftBack},
                    {Drivetrain::mmtrRightFront, Drivetrain::mmtrRightBack})
        .withDimensions(AbstractMotor::gearset::green,
                        {{def::DRIVE_WHEEL_DIAMETER, def::DRIVE_OFFSET}, imev5GreenTPR})
        .build(); // chassis object for using Pathfilder through okapi

/* ----------------------------------------------------------- */
/*                    Protected Information                    */
/* ----------------------------------------------------------- */

/* --------------------- Initial Settings -------------------- */
double Drivetrain::mmaxSpeed = def::SET_DT_MAX;
bool Drivetrain::menabled = true;

/* -------------------- Simple Follow Data ------------------- */
double Drivetrain::mlastLookIndex = 0; // index of the last lookahead point
double Drivetrain::mlastPartialIndex =
    0; // fractional index of where the last lookahead point was on the segment

/* -------------------- Odometry Accessors ------------------- */
OdomState Drivetrain::getState() // get position as OdomState
{
    return CustomOdometry::getState();
}
QLength Drivetrain::getXPos() { return CustomOdometry::getX(); }
QLength Drivetrain::getYPos() { return CustomOdometry::getY(); }
QAngle Drivetrain::getTheta() { return CustomOdometry::getTheta(); }
ExtendedPoint Drivetrain::getPoint() // get position as ExtendedPoint
{
    return ExtendedPoint(getXPos(), getYPos(), getTheta());
}

/* ------------------------- Helpers ------------------------- */
QAngle Drivetrain::angleToPoint(
    const Point & itargetPoint) // calculates the field centric direction to the itargetPoint from
                                // the robot's current position
{
    return (atan((getYPos().convert(inch) - itargetPoint.y.convert(inch)) /
                 (getXPos().convert(inch) - itargetPoint.x.convert(inch))) +
            (getXPos() > itargetPoint.x ? M_PI : 0)) *
           radian;
}
std::optional<double> Drivetrain::findIntersection(
    ExtendedPoint istart, ExtendedPoint iend,
    const double & ilookDistIn) // looks for interections between the line segment created by the
                                // two points (istart and iend), and the circle around the robot
                                // with radius ilookDistIn (lookahead circle)
{
    ExtendedPoint d = iend - istart; // differece vector
    ExtendedPoint f = istart - getPoint(); // robot position relative to the start of the segment

    double a = d.dot(d).convert(inch); // set up quadratic
    double b = 2 * f.dot(d).convert(inch);
    double c = f.dot(f).convert(inch) - ilookDistIn * ilookDistIn;

    double discriminant = b * b - 4 * a * c; // used to make sure it doesn't sqrt(a negative number)
    if (discriminant >= 0)
    {
        discriminant = sqrt(discriminant);

        double t1 = (-b - discriminant) / (2 * a); // solution 1
        double t2 = (-b + discriminant) / (2 * a); // solution 2

        if (t2 >= 0 && t2 <= 1) // t2 is always farther along the segment, so return t2 first
        {
            return t2;
        }
        else if (t1 >= 0 && t1 <= 1) // then t1
        {
            return t1;
        }
    }

    return {}; // no intersections
}
ExtendedPoint Drivetrain::findLookahead(
    SimplePath ipath, const double & ilookDistIn) // looks for the intersection point between the
                                                  // lookahead circle and the SimplePath, ipath
{
    ExtendedPoint currentPos = getPoint();
    int lastIntersectIndex = 0;

    if (currentPos.dist(ipath.last()).convert(inch) <=
        ilookDistIn) // if the last point is within range, return
    {
        return ipath.last();
    }

    for (int i = mlastLookIndex; i < ipath.size() - 1;
         i++) // searchs through the whole path starting at the index of the previous lookhead point
    {
        std::optional<double> t_partialIndex = findIntersection(
            ipath.at(i), ipath.at(i + 1),
            ilookDistIn); // finds the partial index of the intersection in the range [0, 1)
        if (t_partialIndex.has_value() &&
            (i > mlastLookIndex ||
             t_partialIndex > mlastPartialIndex)) // if there is an intersection farther along the
                                                  // path than the last point
        {
            mlastLookIndex = i;
            mlastPartialIndex = t_partialIndex.value();

            if (lastIntersectIndex > 0) // if this is the second intersection, the loop can exit
            {
                break;
            }

            lastIntersectIndex = i; // if this is the first intersection found, record it
        }

        if (lastIntersectIndex > 0 &&
            ipath.at(i).dist(ipath.at(lastIntersectIndex)).convert(inch) >=
                ilookDistIn *
                    2) // if it is searching for intersections farther than the diameter of a
                       // lookahead circle, and it has alread found a point, exit the loop. It is
                       // impossible for there to be a second lookahead point more than 2 *
                       // (lookhead distance) away from the first lookahead point
        {
            break;
        }
    }

    ExtendedPoint segmentStart = ipath.at(mlastLookIndex);
    def::display.setMiscData(
        1, "start: " + segmentStart.string() +
               "\nvec: " + (ipath.at(mlastLookIndex + 1) - segmentStart).string() +
               "\npartlIndx: " + std::to_string(mlastPartialIndex) + "\nscaled: " +
               ((ipath.at(mlastLookIndex + 1) - segmentStart) * mlastPartialIndex).string());
    return segmentStart +
           (ipath.at(mlastLookIndex + 1) - segmentStart) *
               mlastPartialIndex; // calculates the location of the lookahead point by getting the
                                  // vector from the start of the segment to the end of the segment,
                                  // multiplying that by the fractional index of the lookahead
                                  // point, and adding that the the starting point of the segment
}

/* ----------------------------------------------------------- */
/*                      Public Information                     */
/* ----------------------------------------------------------- */
std::shared_ptr<AsyncMotionProfileController> Drivetrain::mprofiler =
    AsyncMotionProfileControllerBuilder()
        .withLimits({def::DRIVE_MAX_SPEED.convert(mps), def::DRIVE_MAX_ACCEL.convert(mps2), 8.0})
        .withOutput(Drivetrain::mchassis)
        .buildMotionProfileController(); // okapi motion profile controller with measured constants

/* --------------------- Getters/Setters --------------------- */
double Drivetrain::getMaxSpeed() { return mmaxSpeed; }
void Drivetrain::setMaxSpeed(double imaxSpeed) { mmaxSpeed = imaxSpeed; }

bool Drivetrain::isEnabled() { return menabled; }
void Drivetrain::enable() // allows movements to be startable
{
    menabled = true;
}
void Drivetrain::disable() // stops active movemnts
{
    menabled = false;
    moveTank(0, 0, false);
}

void Drivetrain::checkNextAsync(
    const double & ierror,
    std::vector<AsyncAction> & iactions) // checks if the next AsyncAction should execute, and
                                         // executes it (and removes it from the list) if it should
{
    if (iactions.size()) // if there is at least one action to execute
    {
        const AsyncAction & nextAction = iactions.at(0);
        if (ierror < nextAction.merror) // if the robot is close enough to the target
        {
            nextAction.maction(); // execute the action
            iactions.erase(iactions.begin()); // remove the action, having already executed it
        }
    }
}

/* ---------------------- Basic Movement ---------------------  /
 * These "basic" motions are lower level fuctions mostly just intended
 * to prevent the call to each motor individually in more advanced
 * motions, to keep the code cleaner.
 *
 * "Saturation" is when the motor inputs are higher than their max
 * speed, which makes the motor go at max speed. This can cause problems,
 * however, when the motors are all working together to follow a
 * specific motion, because one motor might be going at exactly the
 * intended speed, but another motor might be saturated, so it doesn't
 * go at the right speed, making the robot follow the wrong motion.
 * To account for this, the basic movment methods have a variable,
 * idesaturate, that, when true, scales all motor values down so they
 * fit within the motors capabilities.
 */
void Drivetrain::moveIndependant(
    double ileftFront, double irightFront, double irightBack, double ileftBack,
    const bool idesaturate) // moves each motor {lf, rf, rb, lb} in range [-1,1]
{
    if (idesaturate) // desaturates values
    {
        std::array<double, 4> motor_values =
            util::scaleToFit<4>(mmaxSpeed, {ileftFront, irightFront, irightBack, ileftBack});
        ileftFront = motor_values[0];
        irightFront = motor_values[1];
        irightBack = motor_values[2];
        ileftBack = motor_values[3];
    }
    // moves all of the motors by voltage
    mmtrLeftFront.moveVoltage(12000 * ileftFront);
    mmtrRightFront.moveVoltage(12000 * irightFront);
    mmtrRightBack.moveVoltage(12000 * irightBack);
    mmtrLeftBack.moveVoltage(12000 * ileftBack);
}
void Drivetrain::moveTank(double ileft, double iright,
                          const bool idesaturate) // spins the left side and right side motors at
                                                  // certian speeds in range [-1,1]
{
    if (idesaturate) // desaturates values
    {
        std::array<double, 2> motor_values = util::scaleToFit<2>(mmaxSpeed, {ileft, iright});
        ileft = motor_values[0];
        iright = motor_values[1];
    }
    Drivetrain::moveIndependant(
        ileft, iright, iright, ileft,
        false); // don't try to desaturate, because the values have already been desaturated
}
void Drivetrain::moveArcade(
    double iforward, double istrafe, double iturn,
    const bool idesaturate) // moves the robot with arcade-style inputs in range [-1,1]
{
    if (idesaturate) // desaturates values
    {
        std::array<double, 4> motor_values = {
            iforward + istrafe + iturn, iforward - istrafe - iturn, iforward + istrafe - iturn,
            iforward - istrafe + iturn};
        util::scaleToFit<4>(mmaxSpeed, motor_values); // modifies reference to motor_values
        Drivetrain::moveIndependant(motor_values[0], motor_values[1], motor_values[2],
                                    motor_values[3]); // moves the motors from within the if to
                                                      // prevent the need to copy values
    }
    else
    {
        Drivetrain::moveIndependant(iforward + istrafe + iturn, iforward - istrafe - iturn,
                                    iforward + istrafe - iturn, iforward - istrafe + iturn,
                                    false); // don't desaturate
    }
}

/* ------------------ Intermediate Movement ------------------ */
void Drivetrain::moveInDirection(
    QAngle idirection, const bool ifieldCentric, double imagnitude, double itheta,
    const bool idesaturate) // moves the robot with a certain speed in a certain direction, while
                            // turning a certain amount
{
    if (ifieldCentric) // if the direction is in reference to the field
    {
        idirection -= Drivetrain::getTheta(); // changes the direction the robot should go in based
                                              // on its field centric rotation
    }
    idirection = util::wrapQAngle(idirection); // fits idirection into [0, 360)
    util::chop<double>(0, 1, imagnitude); // caps magnitude
    util::chop<double>(-1, 1, itheta); // caps itheta

    Drivetrain::moveArcade(
        imagnitude * cos(idirection.convert(radian)), imagnitude * sin(idirection.convert(radian)),
        itheta, idesaturate); // move in the direction of the vector, and turn the specified amount
}

/* ------------------ Move to Point Methods ------------------  /
 * Because these methods have a target, they need to be run in a loop
 * to constantly re-evaluate how fast the robot should be going. To
 * do this, there are PID and Slew calculations being done for driving
 * straight and turning in each of the methods. This is easy to do
 * because of the PID and Slew classes.
 *
 * Each method has custom tuned default PID/Slew values, but they
 * can be modified on a per-motion basis when they are called.
 */
void Drivetrain::strafeToPoint(
    ExtendedPoint itarget, std::vector<AsyncAction> iactions, PID imagnitudePID, PID iturnPID,
    Slew imagnitudeSlew,
    Slew iturnSlew) // drives in a stright line to the point while turning using set PID/Slew gains,
                    // and executing the AsyncActions at the right times
{
    enable(); // make sure the action can run
    while (menabled && (!imagnitudePID.isSettled() || !iturnPID.isSettled()))
    {
        double inToPoint =
            OdomMath::computeDistanceToPoint(itarget, Drivetrain::getState())
                .convert(inch); // calc inches to target point. itarget can be passed as
                                // okapi::Point, because ExtendedPoint inherits from okapi::Point
        double degToPoint = util::wrapDeg180(
            (itarget.theta - Drivetrain::getTheta())
                .convert(degree)); // calc the angle to the point in the range [-180, 180) to always
                                   // turn the right direction
        def::display.setMiscData(1, std::to_string(degToPoint));

        Drivetrain::moveInDirection(Drivetrain::angleToPoint(itarget), true,
                                    imagnitudeSlew.iterate(imagnitudePID.iterate(inToPoint)),
                                    iturnSlew.iterate(iturnPID.iterate(degToPoint)), true);

        Drivetrain::checkNextAsync(
            inToPoint,
            iactions); // executes the next action if availible, and removes it from the list

        pros::delay(20);
    }
}

void Drivetrain::straightToPoint(
    ExtendedPoint itarget, std::vector<AsyncAction> iactions, QLength inoTurnRange,
    double iturnWeight, PID imagnitudePID, PID iturnPID, Slew imagnitudeSlew,
    Slew iturnSlew) // drives to the point without strafing using set PID/Slew gains, and executing
                    // the AsyncActions at the right times
{
    const double noTurnRangeIn = inoTurnRange.convert(inch);

    enable(); // make sure the action can run
    while (menabled && !imagnitudePID.isSettled())
    {
        QAngle angleToPoint = util::wrapQAngle180(
            Drivetrain::angleToPoint(itarget) -
            Drivetrain::getTheta()); // how much the robot needs to turn to face the point
        double inToPoint = OdomMath::computeDistanceToPoint(itarget, Drivetrain::getState())
                               .convert(inch); // how far the robot is away from the target
        double inForward =
            inToPoint *
            cos(angleToPoint.convert(radian)); // how far the robot needs to drive straight (no
                                               // turning) to get as close to the target as possible

        double forward = imagnitudeSlew.iterate(
            imagnitudePID.iterate(inForward)); // calculates value from PID fed into Slew
        util::chop<double>(-mmaxSpeed, mmaxSpeed,
                           forward); // limits the values in [-mmaxSpeed, mmaxSpeed]

        double turn;
        if (inToPoint > noTurnRangeIn) // if the robot is far enough away from the target
        {
            turn = iturnSlew.iterate(iturnPID.iterate(
                angleToPoint.convert(degree))); // calculates value from PID fed into Slew
            util::chop<double>(-mmaxSpeed, mmaxSpeed, turn);
        }
        else
        {
            turn = 0; // don't turn when too close to the target
        }

        if (abs(turn) == mmaxSpeed) // if the robot is turning at max speed (which means it must be
                                    // far off target)
        {
            turn *= iturnWeight; // increase the amount to turn, so that it turns faster as a result
                                 // of forward getting scaled down in moveArcade
        }

        Drivetrain::checkNextAsync(
            inToPoint,
            iactions); // executes the next action if availible, and removes it from the list

        Drivetrain::moveArcade(forward, 0, turn, true);

        pros::delay(20);
    }
}

void Drivetrain::arcStraightToPoint(
    ExtendedPoint itarget, std::vector<AsyncAction> iactions, double iweightModifier,
    QLength inoTurnRange, PID imagnitudePID, PID iturnPID, Slew imagnitudeSlew,
    Slew iturnSlew) // drive in an "arc" (doesn't follow a path, just approximates an arc) using set
                    // PID/Slew gains, and executing the AsyncActions at the right times
{
    const double noTurnRangeIn = inoTurnRange.convert(inch);

    enable(); // make sure the action can run
    while (menabled && !imagnitudePID.isSettled())
    {
        double theta = util::wrapRad(
            2 * (Drivetrain::getTheta().convert(radian) -
                 abs(atan2(
                     (Drivetrain::getYPos() - itarget.y).convert(inch),
                     (Drivetrain::getXPos() - itarget.x)
                         .convert(inch))))); // calculates how much the robot should end up turning
        double radius = abs(
            hypot((getXPos() - itarget.x).convert(inch), (getYPos() - itarget.y).convert(inch)) /
            2 / sin(theta / 2)); // calculates the radius of the arc
        double targetIn = theta * radius; // how far the robot needs to go (arc length)
        double turnWeight =
            iweightModifier /
            radius; // how aggressively the robot needs to turn to approximate the arc

        QAngle angleToPoint =
            Drivetrain::angleToPoint(itarget) - Drivetrain::getTheta(); // direction of target
        double inToPoint = OdomMath::computeDistanceToPoint(itarget, getState())
                               .convert(inch); // distance of target
        double inForward =
            inToPoint *
            cos((angleToPoint)
                    .convert(radian)); // distance to perpendicular line intersecting target

        double forward = imagnitudeSlew.iterate(
            imagnitudePID.iterate(inForward)); // calculates value from PID fed into Slew
        util::chop<double>(-mmaxSpeed, mmaxSpeed,
                           forward); // limits the values in [-mmaxSpeed, mmaxSpeed]

        double turn;
        if (inToPoint > noTurnRangeIn) // if the robot is far enough away from the target
        {
            turn = iturnSlew.iterate(iturnPID.iterate(
                angleToPoint.convert(degree))); // calculates value from PID fed into Slew
            util::chop<double>(-mmaxSpeed, mmaxSpeed, turn);
        }
        else
        {
            turn = 0; // don't turn when too close to the target
        }

        if (abs(turn) == mmaxSpeed) // if the robot is turning at max speed (which means it must be
                                    // far off target)
        {
            turn *= turnWeight; // increase the amount to turn, so that it turns faster as a result
                                // of forward getting scaled down in moveArcade
        }

        Drivetrain::checkNextAsync(
            inToPoint,
            iactions); // executes the next action if availible, and removes it from the list

        Drivetrain::moveArcade(forward, 0, turn, true);

        pros::delay(20);
    }
}

/* ------------------ Path Following Methods -----------------  /
 * simpleFollow uses the concept from "Pure Pursuit" of using a "lookahead circle" to follow a path.
 * The idea is that, when given a line, the robot will figure out how to follow it smoothly. It does
 * this by checking for points on the line that are a certain distance away from the robot, and
 * moving in the direction of whichever point it sees that is farthest on the line. Another way  to
 * picture this, is the robot has a circle (the lookahead circle) around it with the radius being
 * the "lookahead distance". The robot is always trying to drive to intersections between this
 * circle, and the path (a.k.a. the lookahead point).
 *
 * The robot goes at full speed to the lookahead point until it gets to the end, where it settles
 * with PID.
 */
void Drivetrain::simpleFollow(
    SimplePath ipath, QLength ilookDist, std::vector<AsyncAction> iactions, PID imagnitudePID,
    PID iturnPID, Slew imagnitudeSlew,
    Slew iturnSlew) // follows the path, ipath using set lookahead distance (ilookDist) and PID/Slew
                    // gains while executing the AsyncActions at the right times (only on the last
                    // segment)
{
    double lookDistIn = ilookDist.convert(inch);
    ExtendedPoint lookPoint = ipath.at(0);

    double magnitude = 1; // the robot will always go full speed until the end is near
    bool reachedEnd = false;

    mlastLookIndex = 0;
    mlastPartialIndex = 0;

    enable(); // make sure the action can run
    while (menabled && (!imagnitudePID.isSettled() || !iturnPID.isSettled()) || !reachedEnd)
    {
        if (!reachedEnd &&
            mlastLookIndex ==
                ipath.size() - 2) // detects if the robot should be going to the last point
        {
            reachedEnd = true;
            lookPoint = ipath.last();
        }

        if (!reachedEnd)
        {
            lookPoint = findLookahead(ipath, lookDistIn); // find the next lookahead
        }
        else
        {
            double inToPoint =
                OdomMath::computeDistanceToPoint(lookPoint, Drivetrain::getState())
                    .convert(
                        inch); // calc inches to target point. itarget can be passed as
                               // okapi::Point, because ExtendedPoint inherits from okapi::Point

            Drivetrain::checkNextAsync(
                inToPoint,
                iactions); // executes the next action if availible, and removes it from the list

            magnitude = imagnitudePID.iterate(inToPoint); // how fast the robot should be moving
        }

        double degToPoint = util::wrapDeg180(
            (lookPoint.theta - Drivetrain::getTheta())
                .convert(degree)); // calc the angle to the point in the range [-180, 180) to always
                                   // turn the right direction

        moveInDirection(Drivetrain::angleToPoint(lookPoint), true,
                        imagnitudeSlew.iterate(magnitude),
                        iturnSlew.iterate(iturnPID.iterate(degToPoint)), true);

        pros::delay(20);
    }
}

/* --------------------- Motion Profiling -------------------- */
void Drivetrain::generatePathToPoint(
    PathfinderPoint ipoint,
    const std::string & iname) // use Pathfinder through okapi to make a motion profile
{
    ipoint.y = -ipoint.y;
    ipoint.theta = -ipoint.theta;
    mprofiler->generatePath({{0_ft, 0_ft, 0_deg}, ipoint}, iname);
}
void Drivetrain::followPathfinder(const std::string & iname, bool ibackwards,
                                  bool imirrored) // follow Pathfinder path through okapi
{
    mprofiler->setTarget(iname, ibackwards, imirrored);
}
void Drivetrain::followTraj(Trajectory & itraj) // follow trajectory loaded from sd card
{
    const double startLeft = mmtrLeftFront.getPosition();
    const double startRight = mmtrRightFront.getPosition();

    if (itraj.isDone()) // if the path is done before execution, reset
    {
        itraj.reset();
    }
    while (!itraj.isDone()) // execute until the path is done
    {
        std::pair<double, double> values = itraj.iterate(
            (mmtrLeftFront.getPosition() - startLeft) * def::DRIVE_WHEEL_CIRCUMFERENCE_IN / 360,
            (mmtrRightFront.getPosition() - startRight) * def::DRIVE_WHEEL_CIRCUMFERENCE_IN /
                360); // iterate through the profile passing the distance each side has gone so far

        moveTank(values.first, values.second, false);

        pros::delay(10);
    }
}