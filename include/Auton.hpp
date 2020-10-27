/**
 * Auton.hpp
 *
 * This contains the declaration of the Auton struct,
 * which is responsible for reading the sd card to determine
 * which auton is selected, and running the correct auton.
 */
#pragma once // makes sure the file is only included once
#include "main.h" // gives access to dependancies from other files
class Auton
{
  public:
    // when making autons, you must add the text to the dropdown in DisplayControl.cpp, a new value
    // to this enum, and a new case is the switch in Auton.cpp
    enum class Autons // all possible autons
    {
        none,
        test,
        prog
    } static auton; // selected auton

    static void readSettings(); // read the sd card to set the settings

    static void runAuton(); // runs the selected auton

  private:
    static void auton_task_func(void *); // separate thread for running the auton, in case a
                                         // particular auton needs control over it's thread.
};