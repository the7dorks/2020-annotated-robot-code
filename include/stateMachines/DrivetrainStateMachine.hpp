/**
 * DrivetrainStateMachine.hpp
 *
 * This file contains the declaration of the DrivetrainStateMachine class.
 * DrivetrainStateMachine is a state machine that inherits from VStateMachine.
 * It has an enumeration of different possible states to make it easy for
 * the user to controll the drivetrain.
 *
 * To use the state machine in auton, you use doAutonMotion() to disable
 * the normal state machine tasks and run the specified action.
 */
#pragma once // makes sure the file is only included once
#include "main.h" // gives access to dependancies from other files
class DrivetrainStateMachine : public VStateMachine // state machine to represent the drivetrain
{
  public:
    DrivetrainStateMachine(); // constructor to set defaults
    enum class MStates // enumeration to organize possible states
    {
        off,
        busy, // doing an AutonMotion
        manual, // standard split arcade drive
        fieldCentric // standard split arcade, but with field centric turns
    };
    MStates getState();
    void setState(MStates istate);

    void
    doAutonMotion(std::function<void()> iaction); // disable manual control, and execute the action

    void controlState() override; // update the state based on controller input
    void update() override; // move the robot based on the state

  private:
    /* -------------------------- State -------------------------- */
    MStates mstate, mlastState;

    bool stateChanged(); // returns whether the last state is the same as the current one

    /* ------------------------- Controls ------------------------ */
    Controller & mcontroller; // reference to the controller to get joystick values
    ControllerButton &
        mtoggleFieldCentric; // reference to the button that toggles field centric control

    /* -------------------------- Other -------------------------- */
    Drivetrain & mdrivetrain; // reference to the drivetrain to give control of the drivetrain to
                              // the state machine
};

namespace def
{
extern DrivetrainStateMachine
    sm_dt; // declares the sm_dt object as extern, to make sure it only gets constructed once
} // namespace def
