/**
 * main.cpp
 *
 * This file contains the orchestration of all the compenents. It
 * starts all of the separate tasks that are needed for controlling
 * the robot, and has all the functions called by the competition
 * switch.
 */
#include "main.h" // gives access to dependencies from other files

DisplayControl def::display = DisplayControl();
pros::Task sm_dt_task(sm_dt_task_func);
pros::Task sm_bc_task(sm_bc_task_func);
pros::Task odomTask(odomTaskFunc);
pros::Task display_task(display_task_func);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
    Auton::readSettings(); // read sd card to remeber the auton selected when the brain was run last
    def::display.setAutonDropdown(); // update auton dropdown to match the sd card

    def::mtr_dt_left_front.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    def::mtr_dt_right_front.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    def::mtr_dt_right_back.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    def::mtr_dt_left_back.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    def::mtr_it_left.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    def::mtr_it_right.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    def::mtr_ix.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    def::mtr_fw1.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    def::mtr_fw2.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{
    Auton::runAuton(); // uses the auton class to run the slected auton
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{

    def::sm_dt.setState(
        DT_STATES::manual); // set the drivetrain to basic controls during drivercontrol

    // there is no need for a loop in opcontrol(), because there are already other tasks running
    // that control all of the movement
    // while (true)
    // {
    //     pros::delay(20);
    // }
}
