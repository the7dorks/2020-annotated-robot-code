/**
 * definitions.cpp
 *
 * This file contains various declarations and definitions for
 * motors, sensors, controls, constants, and settings, so that
 * things that might need to be changed are all in one place.
 */
#include "main.h" // gives access to definition.hpp and other dependencies

namespace def
{
/* ----------------------------------------------------------- */
/*                           Devices                           */
/* ----------------------------------------------------------- */

/* -------------------------- Motors ------------------------- */
Motor mtr_dt_left_front(16);
Motor mtr_dt_right_front(-1);
Motor mtr_dt_right_back(-19);
Motor mtr_dt_left_back(20);
/* ----------------------------------------------------------- */
Motor mtr_it_left(18);
Motor mtr_it_right(-3);
/* ----------------------------------------------------------- */
Motor mtr_ix(2);
/* ----------------------------------------------------------- */
Motor mtr_fw1(-17);
Motor mtr_fw2(7);

/* ------------------------- Sensors ------------------------- */
ADIEncoder track_encoder_forward('G', 'H', true);
ADIEncoder track_encoder_side('E', 'F', true);
pros::Imu imu_top(4);
pros::Imu imu_bottom(5);

/* ----------------------------------------------------------- */
/*                           Controls                          */
/* ----------------------------------------------------------- */
Controller controller = Controller();

/* ------------------------ Drivetrain ----------------------- */
ControllerButton btn_dt_tglFieldCentric = ControllerDigital::A;

/* ----------------------- Ball Control ---------------------- */
ControllerButton btn_bc_in = ControllerDigital::R1;
ControllerButton btn_bc_out = ControllerDigital::R2;
ControllerButton btn_bc_shoot = ControllerDigital::L1;
ControllerButton btn_bc_down = ControllerDigital::L2;

/* ----------------------------------------------------------- */
/*                          Constructs                         */
/* ----------------------------------------------------------- */
CustomOdometry customOdom = CustomOdometry(); // object that calculates position

Drivetrain drivetrain = Drivetrain(); // used by DrivetrainStateMachine for drivetrain control

DrivetrainStateMachine sm_dt = DrivetrainStateMachine(); // state machine to control the drivetrain
BallControlStateMachine sm_bc = BallControlStateMachine(); // state machine for ball manipulators
} // namespace def