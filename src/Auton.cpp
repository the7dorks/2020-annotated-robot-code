/**
 * Auton.cpp
 *
 * This contains the definitions of the Auton struct,
 * which is responsible for reading the sd card to determine
 * which auton is selected, and running the correct auton.
 */
#include "main.h" // gives access to Auton and other dependencies

Auton::Autons Auton::auton =
    Auton::Autons::none; // default auton is none, if the sd card is not installed

void Auton::readSettings() // read the sd card to set the settings
{
    FILE * file; // cpp a file object to be used later
    if (pros::usd::is_installed()) // checks if the sd card is installed before trying to read it
    {
        file = fopen("/usd/auton_settings.txt", "r"); // open the auton settings
        if (file) // check to see if the file opened correctly
        {
            fscanf(file, "%i", &auton);
        }
        else
        {
            std::cout << "/usd/auton_settings.txt is null."
                      << std::endl; // if the file didn't open right, tell the terminal
        }
        fclose(file); // close the file
    }
}

void Auton::runAuton() // runs the selected auton
{
    pros::Task auton_task(auton_task_func);
}
/*
  ____
 /    \
|   1  |
 \____/

*/

/* ----------------------------------------------------------- */
/*                     Private Information                     */
/* ----------------------------------------------------------- */
void Auton::auton_task_func(void *) // separate thread for running the auton, in case a particular
                                    // auton needs control over its thread
{
    // when making autons, you must add the text to the dropdown in DisplayControl.cpp, a new enum
    // value in Auton.hpp, and a new case is this switch
    switch (auton)
    {
        case Autons::none:
            break;
        case Autons::test:
            def::sm_dt.doAutonMotion(makeFunc({
                def::drivetrain.strafeToPoint({4_ft, 0_ft, 0_deg});
                def::drivetrain.strafeToPoint({4_ft, -3.8_ft, 180_deg});
                def::drivetrain.strafeToPoint({4_in, -3.8_ft, 0_deg});
                def::drivetrain.strafeToPoint({4_in, -4_in, 0_deg});
            }));
            break;
        case Autons::prog:
            /**
             * `makeFunc()` is a preprocessor macro that takes the contents and turns them into a
             * lambda function, [](){}. It is frequently used to specify the actions in
             * AcyncActions.
             *
             * `cutDrive()` is a preprocessor macro that adds an AsyncAction to the motion that
             * disables the motion at a certain error (in inches) from the target. This is used
             * frequently because many motions do not need to go exactly to the target. When
             * possible, stopping the motion before it reaches the target is faster, because it
             * doesn't need to use PID to settle.
             */
            CustomOdometry::setState({8_in, 37_in, 0_deg}); // set the starting position
            def::sm_dt.doAutonMotion(makeFunc({
                deploy(); // deploys the hood
                /* -------------------------- Goal 1 ------------------------- */
                def::sm_bc.ixUp(); // start the indexer to get the first ball
                def::drivetrain.strafeToPoint(
                    {17_in, 36_in, 0_deg}, // get the first ball (now 2 balls)
                    cutDrive(2));
                def::drivetrain.strafeToPoint({15_in, 16_in, 225_deg}, // line up with goal #1
                                              cutDrive(1.5));
                def::sm_bc.ixOff(); // turns off the indexer
                pros::delay(300);
                def::sm_bc.shoot(); // now 1 ball

                /* -------------------------- Goal 2 ------------------------- */
                def::drivetrain.strafeToPoint(
                    {35_in, 24_in, -90_deg}, // line up with the second ball
                    cutDrive(2));
                def::sm_bc.ixUp(); // get ready for the next ball by starting the indexer
                def::drivetrain.strafeToPoint({35_in, 10_in, -90_deg}); // get the next ball (now 2)
                def::drivetrain.strafeToPoint({73_in, 26_in, 0_deg}, // get the next ball (now 3)
                                              cutDrive(2));
                def::drivetrain.strafeToPoint(
                    {73_in, 29_in, -90_deg}, {}, PID(0.4, 0.005, 2.6, 0.5, 0.5, 0.5, 1_ms),
                    PID(0.028, 0.0, 0.08, 0.0, 5, 2,
                        1_ms)); // turn to face the goal with custom PID gains, becuase for some
                                // reason, this specific motion frequently settled inconsistantly
                def::drivetrain.strafeToPoint({71_in, 20_in, -90_deg}, // drive to the next goal
                                              cutDrive(1));
                def::sm_bc.ixOff(); // turns off the indexer
                def::sm_bc.shoot(); // now 2 balls

                /* -------------------------- Goal 3 ------------------------- */
                def::drivetrain.strafeToPoint(
                    {108_in, 34_in, 0_deg},
                    {AsyncAction(10,
                                 makeFunc({ def::sm_bc.ixUp(); })), // start the indexer mid-motion
                     AsyncAction(2, makeFunc({ def::drivetrain.disable(); }))}); // same cutDrive(2)
                def::drivetrain.strafeToPoint({118_in, 34_in, 0_deg}, cutDrive(1));
                def::drivetrain.strafeToPoint({126_in, 14_in, -36_deg}, // line up with goal #3
                                              cutDrive(1));
                def::sm_bc.ixOff(); // turn off the indexer
                def::sm_bc.shoot(); // now 2 balls

                /* -------------------------- Goal 4 ------------------------- */
                def::drivetrain.strafeToPoint({114_in, 72_in, 0_deg}, // move towards goal #4
                                              cutDrive(3));
                def::sm_bc.ixUp();
                def::drivetrain.strafeToPoint({125_in, 72_in, -2_deg}, // line up with goal #4
                                              cutDrive(1));
                def::sm_bc.ixOff(); // turs the indexer off
                def::sm_bc.shoot(); // now 1 ball
                def::sm_bc.ixUp(); // get ready to shoot the next ball
                pros::delay(700);
                def::sm_bc.shoot(); // shoot now 0 balls
                def::sm_bc.ixOff(); // turns the indexer off

                /* -------------------------- Goal 5 ------------------------- */
                def::drivetrain.strafeToPoint({122_in, 90_in, 90_deg}, // move towards the next ball
                                              cutDrive(2));
                def::sm_bc.ixUp(); // turns the indexer on
                def::drivetrain.strafeToPoint(
                    {123_in, 108_in, 90_deg}, // get the next ball (now 1 ball)
                    cutDrive(1));
                def::drivetrain.strafeToPoint({120_in, 124_in, 45_deg}, // move towards goal #5
                                              cutDrive(3));
                def::drivetrain.strafeToPoint({130_in, 126_in, 43_deg}, // line up with goal #5
                                              cutDrive(1));
                def::sm_bc.shoot(500); // now 0 balls
                def::sm_bc.ixOff(); // turns the indexer off

                /* -------------------------- Goal 6 ------------------------- */
                def::sm_bc.ixUp(); // turns the indexer on
                def::drivetrain.strafeToPoint(
                    {85_in, 120_in, 180_deg}, // move towards the next ball
                    cutDrive(2));
                def::drivetrain.strafeToPoint({70_in, 118_in, 180_deg}, // get the next ball (now 1)
                                              cutDrive(1));
                def::drivetrain.strafeToPoint({77_in, 123_in, 90_deg},
                                              cutDrive(0.25)); // line up with the goal
                def::sm_bc.shoot(600); // now 0
                def::sm_bc.ixOff(); // turns off the indexer
                pros::delay(100); // pause to make sure the shot works

                /* -------------------------- Goal 7 ------------------------- */
                def::sm_bc.ixUp(); // get ready for the next ball
                def::drivetrain.strafeToPoint({37_in, 124_in, 90_deg}, // line up with the next ball
                                              cutDrive(3));
                def::drivetrain.strafeToPoint({38_in, 136_in, 90_deg}); // get the next ball (now 1)
                def::drivetrain.strafeToPoint({17.5_in, 125.5_in, 119_deg}, // line up with goal #7
                                              cutDrive(1));
                def::sm_bc.shoot(600); // now 0 balls
                def::sm_bc.ixOff(); // turns off the indexer

                /* -------------------------- Goal 8 ------------------------- */
                def::drivetrain.strafeToPoint({28_in, 129_in, -90_deg}); // turn around
                def::sm_bc.ixUp(); // turns the indexer on
                def::drivetrain.strafeToPoint(
                    {28_in, 119_in, -90_deg}, // gets the next ball (now 1)
                    cutDrive(2));
                def::drivetrain.strafeToPoint({28_in, 69_in, 180_deg}, // move towards goal #8
                                              cutDrive(3));
                def::drivetrain.strafeToPoint({21_in, 72_in, 180_deg}, // line up with goal #8
                                              cutDrive(1));
                def::sm_bc.shoot(600); // now 0
                def::sm_bc.ixOff();

                /* -------------------------- Goal 9 ------------------------- */
                def::drivetrain.strafeToPoint({23_in, 72_in, 0_deg}, cutDrive(0.5)); // turn around
                def::sm_bc.ixUp(); // get ready to get the ball by turning the indexer on
                def::drivetrain.strafeToPoint({48_in, 72_in, 0_deg}, // get the next ball (now 1)
                                              cutDrive(1));
                def::drivetrain.strafeToPoint(
                    {36_in, 72_in, 0_deg}, // back up to make sure the descorer doesn't hit the goal
                    cutDrive(2));
                def::sm_bc.itOut(); // deploy
                pros::delay(600); //
                def::sm_bc.itOff(); // stop the descorer
                def::drivetrain.strafeToPoint({56_in, 69_in, 0_deg}); // descore
                pros::delay(500);
                def::sm_bc.ixOff(); // stop the indexer
                def::drivetrain.strafeToPoint({30_in, 76_in, 0_deg}, cutDrive(2)); // back up
                def::drivetrain.strafeToPoint({57_in, 79_in, -10_deg}, // go to the goal
                                              cutDrive(1));
                def::sm_bc.shoot(); // now 0
            }));
            break;
    }
}