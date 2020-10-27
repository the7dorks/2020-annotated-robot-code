/**
 * AsyncAction.hpp
 *
 * This file contains the definition of the AsyncAction struct.
 * AsyncActions are objects that have an action (maction) and
 * a certain error where the action should be executed (merror).
 * It is used by motions in the Drivetrain class to run asynchronus
 * actions at a certain distance from the target.
 */
#pragma once // makes sure the file is only included once
#include "main.h" // gives access to objects declared elsewhere
struct AsyncAction
{
    AsyncAction(double ierror, std::function<void()> iaction)
        : merror(ierror), maction(iaction) // constructor
    {
    }

    double merror; // error value at which the loop will execute the action
    std::function<void()> maction; // action to execute
};
