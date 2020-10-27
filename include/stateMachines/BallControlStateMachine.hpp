/**
 * BallControlStateMachine.hpp
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
#pragma once // makes sure the file is only included once
#include "main.h" // gives access to dependencies from other files
class BallControlStateMachine : public VStateMachine // state machine to represent the ball
                                                     // controlers (intake/indexer/flywheel)
{
  public:
    BallControlStateMachine(); // constructor to set defaults

    void controlState() override; // sets the state based on inputs from the controller
    void update() override; // controls the robot based on the state

    void itIn(); // spins the intakes in
    void itOut(); // spins the intakes out
    void itOff(); // stops the intakes
    void ixUp(); // spins the indexer up
    void ixDown(); // spins the indexer down
    void ixOff(); // stops the indexer
    void fwShoot(); // shoots the flywheel
    void fwFilter(); // spins the flywheel backwards
    void fwOff(); // stops the flywheel

    void itInFor(double imilliseconds); // spins the intakes for specified number of miliseconds
    void ixUpFor(double imilliseconds); // spins the indexer for specified number of miliseconds
    void shoot(int ims = 300); // shoots a ball

    bool controlEnabled; // decides if the state machine should pay attention to the controller

  private:
    /* ------------------------- Controls ------------------------ */
    ControllerButton & mbtnIn;
    ControllerButton & mbtnOut;
    ControllerButton & mbtnShoot;
    ControllerButton & mbtnFilter;

    /* ---------------------- Nested Classes --------------------- */
    struct MIntake // controls the intakes
    {
        MIntake(); // constructor to set defaults

        enum MStates // enumeration to organize all possible states
        {
            off,
            in,
            out
        };

        void update(); // updates the subsystem based on the state

        /* -------------------------- State -------------------------- */
        MStates mstate;

        /* -------------------------- Other -------------------------- */
        MotorGroup mmotors;
    } mintake;

    struct MIndexer // controls the indexer
    {
        MIndexer(); // constructor to set defaults

        enum class MStates // enumeration to organize all possible states
        {
            off,
            in,
            out
        };

        void update(); // updates the subsystem based on the state

        /* -------------------------- State -------------------------- */
        MStates mstate;

        /* -------------------------- Other -------------------------- */
        Motor mmotor;
    } mindexer;

    struct MFlywheel // controls the flywheel
    {
        MFlywheel(); // constructor to set defaults

        enum class MStates // enumeration to organize all possible states
        {
            off,
            shoot, // forward
            filter // reverse
        };

        void update(); // updates the subsystem based on the state

        /* -------------------------- State -------------------------- */
        MStates mstate;

        /* -------------------------- Other -------------------------- */
        MotorGroup mmotors;
    } mflywheel;
};

namespace def
{
extern BallControlStateMachine
    sm_bc; // declares the sm_bc object as extern, to make sure it only gets constructed once
} // namespace def