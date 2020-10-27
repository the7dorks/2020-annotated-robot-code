/**
 * BallControlStateMachine.cpp
 *
 * This file contains the definitions of the BallControlStateMachine class.
 * BallControlStateMachine inherits from VStateMachine, and
 * it is responsible for controlling all of the ball manipulators
 * (intake, indexer, filter, and flywheel).
 *
 * The intake, indexer, and flywheel all have their own mini
 * state machine in structs all contained in
 * BallControlStateMachine. BallControlStateMachine puts them
 * all together to make them function cohesively
 */
#include "main.h" // gives access to BallControlStateMachine and other dependencies

/* ----------------------------------------------------------- */
/*                      Public Information                     */
/* ----------------------------------------------------------- */
BallControlStateMachine::BallControlStateMachine()
    : controlEnabled(true), mbtnIn(def::btn_bc_in), mbtnOut(def::btn_bc_out),
      mbtnShoot(def::btn_bc_shoot), mbtnFilter(def::btn_bc_down)
{
} // constructor to set defaults

void BallControlStateMachine::controlState() // sets the mini states based on inputs from the
                                             // controller
{
    if (mbtnOut.changedToPressed()) // when the  out button is pressed
    {
        // spin out
        mintake.mstate = IT_STATES::out;
        mindexer.mstate = IX_STATES::out;
    }
    else if (mbtnOut.changedToReleased()) // when the out button is released
    {
        if (mbtnIn.isPressed()) // if the in button is also being pressed
        {
            // spin in
            mintake.mstate = IT_STATES::in;
            mindexer.mstate = IX_STATES::in;
        }
        else
        {
            // stop spinning
            mintake.mstate = IT_STATES::off;
            mindexer.mstate = IX_STATES::off;
        }
    }

    if (mbtnIn.changedToPressed()) // if the in button is pressed
    {
        // spin in
        mintake.mstate = IT_STATES::in;
        mindexer.mstate = IX_STATES::in;
    }
    else if (mbtnIn.changedToReleased()) // if the in button is released
    {
        if (mbtnOut.isPressed()) // is the out button is also being pressed
        {
            // spin out
            mintake.mstate = IT_STATES::out;
            mindexer.mstate = IX_STATES::out;
        }
        else
        {
            // stop
            mintake.mstate = IT_STATES::off;
            mindexer.mstate = IX_STATES::off;
        }
    }

    if (mbtnFilter.changedToPressed()) // if the filter button is pressed
    {
        mflywheel.mstate = FW_STATES::filter; // spin the filter
    }
    if (mbtnFilter.changedToReleased()) // is the filter button is released
    {
        if (mbtnShoot.isPressed()) // if the shoot button is also being pressed
        {
            mflywheel.mstate = FW_STATES::shoot; // shoot
        }
        else
        {
            mflywheel.mstate = FW_STATES::off; // stop spinning
        }
    }

    if (mbtnShoot.changedToPressed()) // if the shoot button is pressed
    {
        mflywheel.mstate = FW_STATES::shoot; // shoot
    }
    else if (mbtnShoot.changedToReleased()) // if the shoot button is released
    {
        if (mbtnFilter.isPressed()) // if the filter button is also being pressed
        {
            mflywheel.mstate = FW_STATES::filter; // spin the filter
        }
        else
        {
            mflywheel.mstate = FW_STATES::off; // stop shooting
        }
    }
}
void BallControlStateMachine::update() // controls the robot based on the state by updating each
                                       // mini state machine independantly
{
    mintake.update();
    mindexer.update();
    mflywheel.update();
}

void BallControlStateMachine::itIn() // spins the intakes in
{
    mintake.mstate = IT_STATES::in;
}
void BallControlStateMachine::itOut() // spins the intakes out
{
    mintake.mstate = IT_STATES::out;
}
void BallControlStateMachine::itOff() // stops the intakes
{
    mintake.mstate = IT_STATES::off;
}
void BallControlStateMachine::ixUp() // spins the indexer up
{
    mindexer.mstate = IX_STATES::in;
}
void BallControlStateMachine::ixDown() // spins the indexer down
{
    mindexer.mstate = IX_STATES::out;
}
void BallControlStateMachine::ixOff() // stops the indexer
{
    mindexer.mstate = IX_STATES::off;
}
void BallControlStateMachine::fwShoot() // shoots the flywheel
{
    mflywheel.mstate = FW_STATES::shoot;
}
void BallControlStateMachine::fwFilter() // spins the flywheel backwards
{
    mflywheel.mstate = FW_STATES::filter;
}
void BallControlStateMachine::fwOff() // stops the flywheel
{
    mflywheel.mstate = FW_STATES::off;
}

void BallControlStateMachine::itInFor(
    double imilliseconds) // spins the intakes for specified number of miliseconds
{
    mintake.mstate = IT_STATES::in;
    pros::delay(imilliseconds);
    mintake.mstate = IT_STATES::off;
}
void BallControlStateMachine::ixUpFor(
    double imilliseconds) // spins the indexer for specified number of miliseconds
{
    mindexer.mstate = IX_STATES::in;
    pros::delay(imilliseconds);
    mindexer.mstate = IX_STATES::off;
}
void BallControlStateMachine::shoot(int ims) // shoots a ball
{
    mflywheel.mstate = FW_STATES::shoot;
    pros::delay(ims);
    mflywheel.mstate = FW_STATES::off;
}
/* ----------------------------------------------------------- */
/*                     Private Information                     */
/* ----------------------------------------------------------- */

/* ---------------------- Nested Classes --------------------- */
BallControlStateMachine::MIntake::MIntake()
    : mstate(IT_STATES::off), mmotors({def::mtr_it_left, def::mtr_it_right})
{
} // constructor to set defaults
void BallControlStateMachine::MIntake::update() // updates the subsystem based on the state
{
    switch (mstate)
    {
        case IT_STATES::off:
            mmotors.moveVoltage(0);
            break;
        case IT_STATES::in:
            mmotors.moveVoltage(12000);
            break;
        case IT_STATES::out:
            mmotors.moveVoltage(-12000);
            break;
    }
}
/* ----------------------------------------------------------- */
BallControlStateMachine::MIndexer::MIndexer()
    : mstate(IX_STATES::off), mmotor(def::mtr_ix) {} // constructor to set defaults
void BallControlStateMachine::MIndexer::update() // updates the subsystem based on the state
{
    switch (mstate)
    {
        case IX_STATES::off:
            mmotor.moveVoltage(0);
            break;
        case IX_STATES::in:
            mmotor.moveVoltage(12000);
            break;
        case IX_STATES::out:
            mmotor.moveVoltage(-12000);
            break;
    }
}

BallControlStateMachine::MFlywheel::MFlywheel()
    : mstate(FW_STATES::off), mmotors({def::mtr_fw1, def::mtr_fw2})
{
} // constructor to set defaults
void BallControlStateMachine::MFlywheel::update() // updates the subsystem based on the state
{
    switch (mstate)
    {
        case FW_STATES::off:
            mmotors.moveVoltage(0);
            break;
        case FW_STATES::shoot:
            mmotors.moveVoltage(12000);
            break;
        case FW_STATES::filter:
            mmotors.moveVoltage(-12000);
            break;
    }
}
