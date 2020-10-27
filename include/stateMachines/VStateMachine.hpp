/**
 * VStateMachine.hpp
 *
 * This file contains the parent class, VStateMachine.
 * VStateMachine is an abstract base class for all state
 * machines. It specifies that all state machines should
 * have a method that controls the state based on user
 * input, and a method that moves the robot based on the state.
 */
#pragma once // makes sure the file is only included once
class VStateMachine // abstract state machine base class
{
  public:
    virtual void controlState() = 0; // changes the state based on user input
    virtual void update() = 0; // controls the subsystem based on the current state
};
