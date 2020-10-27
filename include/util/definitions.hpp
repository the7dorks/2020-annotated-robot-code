/**
 * definitions.hpp
 *
 * This file contains various declarations and definitions for
 * motors, sensors, controls, constants, and settings, so that
 * things that might need to be changed are all in one place.
 */
#pragma once // makes sure the file is only included once
#include "main.h" // gives access to dependancies from other files

#define DT_STATES DrivetrainStateMachine::MStates
#define IT_STATES BallControlStateMachine::MIntake::MStates
#define IX_STATES BallControlStateMachine::MIndexer::MStates
#define FW_STATES BallControlStateMachine::MFlywheel::MStates

#define makeFunc(i) [&]() i
#define cutDrive(i)                                                                                \
    {                                                                                              \
        AsyncAction(i, makeFunc({ def::drivetrain.disable(); }))                                   \
    }
namespace def
{
/* ----------------------------------------------------------- */
/*                           Devices                           */
/* ----------------------------------------------------------- */

/* -------------------------- Motors ------------------------- */
extern Motor mtr_dt_left_front;
extern Motor mtr_dt_right_front;
extern Motor mtr_dt_right_back;
extern Motor mtr_dt_left_back;
/* ----------------------------------------------------------- */
extern Motor mtr_it_left;
extern Motor mtr_it_right;
/* ----------------------------------------------------------- */
extern Motor mtr_ix;
/* ----------------------------------------------------------- */
extern Motor mtr_fw1;
extern Motor mtr_fw2;

/* ------------------------- Sensors ------------------------- */
extern ADIEncoder track_encoder_forward;
extern ADIEncoder track_encoder_side;
extern pros::Imu imu_top;
extern pros::Imu imu_bottom;

/* ----------------------------------------------------------- */
/*                           Controls                          */
/* ----------------------------------------------------------- */
extern Controller controller;

/* ------------------------ Drivetrain ----------------------- */
extern ControllerButton btn_dt_tglFieldCentric;

/* ----------------------- Ball Control ---------------------- */
extern ControllerButton btn_bc_in;
extern ControllerButton btn_bc_out;
extern ControllerButton btn_bc_shoot;
extern ControllerButton btn_bc_down;

/* ----------------------------------------------------------- */
/*                          Constants                          */
/* ----------------------------------------------------------- */

/* --------------------- Tracking Wheels --------------------- */
const QLength TRACK_WHEEL_DIAMETER = 2.847_in;
const QLength TRACK_WHEEL_CIRCUMFERENCE = TRACK_WHEEL_DIAMETER * M_PI;
const QLength TRACK_FORWARD_OFFSET = 2.3_in;
const QLength TRACK_SIDE_OFFSET = 7_in;

/* ------------------------ Drivetrain ----------------------- */
const QLength DRIVE_WHEEL_DIAMETER = 4.041_in;
const double DRIVE_WHEEL_DIAMETER_IN = DRIVE_WHEEL_DIAMETER.convert(inch);
const QLength DRIVE_WHEEL_CIRCUMFERENCE = DRIVE_WHEEL_DIAMETER * M_PI;
const double DRIVE_WHEEL_CIRCUMFERENCE_IN = DRIVE_WHEEL_CIRCUMFERENCE.convert(inch);
const QLength DRIVE_OFFSET = 42_in;

const QAcceleration DRIVE_MAX_ACCEL = 1_G; // approxamate measured linear acceleration
const QSpeed DRIVE_MAX_SPEED = 2.7_mps; // a measured linear velocity

/* ------------------------- Settings ------------------------ */
const double SET_DT_MAX = 1; // default drivetrain max speed (1 is 100%)
const OdomState SET_ODOM_START = {0_ft, 0_ft, 0_deg}; // starting position of the robot on the field
} // namespace def