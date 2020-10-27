/**
 * DrivetrainStateMachine.cpp
 *
 * This file contains the definitions of the DrivetrainStateMachine class.
 * DrivetrainStateMachine is a state machine that inherits from VStateMachine.
 * It has an enumeration of different possible states to make it easy for
 * the user to controll the drivetrain.
 *
 * To use the state machine in auton, you use doAutonMotion() to disable
 * the normal state machine tasks and run the specified action.
 */
#include "main.h" // gives access to DrivetrainStateMachine and other dependencies

/* ----------------------------------------------------------- */
/*                     Private Information                     */
/* ----------------------------------------------------------- */

/* -------------------------- State -------------------------- */
bool DrivetrainStateMachine::stateChanged() // returns whether the last state is the same as the current one
{
    if (mstate != mlastState)
    {
        return true;
    }
    return false;
}

/* ----------------------------------------------------------- */
/*                      Public Information                     */
/* ----------------------------------------------------------- */
DrivetrainStateMachine::DrivetrainStateMachine() : mstate(DT_STATES::off), mlastState(mstate), mcontroller(def::controller), mtoggleFieldCentric(def::btn_dt_tglFieldCentric), mdrivetrain(def::drivetrain) {} // constructor to set defaults

DT_STATES DrivetrainStateMachine::getState() { return mstate; }
void DrivetrainStateMachine::setState(DT_STATES istate)
{
    mlastState = mstate;
    mstate = istate;
}

void DrivetrainStateMachine::doAutonMotion(std::function<void()> iaction) // disable manual control, and execute the action
{
    DT_STATES oldState = mstate;
    setState(DT_STATES::busy);
    iaction();
    setState(oldState);
}

void DrivetrainStateMachine::controlState() // update the state based on controller input
{
    if (mtoggleFieldCentric.changedToPressed()) // toggle field centric
    {
        if (mstate == DT_STATES::manual)
        {
            mstate = DT_STATES::fieldCentric;
        }
        else
        {
            mstate = DT_STATES::manual;
        }
    }
}

void DrivetrainStateMachine::update() // move the robot based on the state
{
    switch (mstate)
    {
        case DT_STATES::off:
            break;
        case DT_STATES::busy:
            break;
        case DT_STATES::manual: // normal, arcade control
            mdrivetrain.moveArcade(mcontroller.getAnalog(ControllerAnalog::leftY), mcontroller.getAnalog(ControllerAnalog::leftX), mcontroller.getAnalog(ControllerAnalog::rightX), false);
            break;
        case DT_STATES::fieldCentric: // field centric arcade control
            QAngle direction = 90_deg - atan2(mcontroller.getAnalog(ControllerAnalog::leftY), mcontroller.getAnalog(ControllerAnalog::leftX)) * radian;
            double magnitude = hypot(mcontroller.getAnalog(ControllerAnalog::leftX), mcontroller.getAnalog(ControllerAnalog::leftY));
            mdrivetrain.moveInDirection(direction, true, magnitude, mcontroller.getAnalog(ControllerAnalog::rightX), true);
            break;
    }
}
