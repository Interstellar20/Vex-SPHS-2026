#include "scoringMacro.h"
#include "lift.h"
#include "scoring.h"

// Scoring Macro Settings

// Turn entire macro on/off
bool scoringMacroEnabled = true;

// How far the cascade moves before scoring mech starts
double macroLiftStartScoringPosition = 450;

// Final cascade position
double macroLiftTargetPosition = 580;

// Final scoring mechanism position
double macroScoringTargetPosition = 600;

// How long the scoring wheels accept during the scoring macro
int macroScoringWheelsTime = 750;

// How close the motors need to be to count as finished
double macroPositionTolerance = 25;

// How close the cascade needs to be to 0 to count as fully down
double macroLiftBottomTolerance = 5;

// How long the scoring wheels release before returning
int macroScoringReleaseTime = 500;

// How much farther the cascade moves up before returning
double macroReturnLiftBumpAmount = 100;

// How fast the robot drives forward before returning
int macroDriveForwardSpeed = 50;

// How long the robot drives forward before returning
int macroDriveForwardTime = 300;

// Maximum amount of time the scoring macro can run
int macroSafetyTimeout = 3000;


// Drive Motors

pros::MotorGroup macroLeftDriveMotors(
    {-15, -12},
    pros::MotorGears::blue
);

pros::MotorGroup macroRightDriveMotors(
    {19, 11},
    pros::MotorGears::blue
);


// Scoring Macro State

enum ScoringMacroState {
    MACRO_IDLE,
    MACRO_RAISING_LIFT,
    MACRO_RAISING_BOTH,
    MACRO_RELEASING,
    MACRO_RETURN_LIFT_BUMP,
    MACRO_DRIVING_FORWARD,
    MACRO_RETURNING_LIFT,
    MACRO_RETURNING_SCORING
};

ScoringMacroState scoringMacroState = MACRO_IDLE;

// false = robot is in resting setup
// true = robot is in scoring setup
bool scoringPositionActive = false;

// Remembers when the scoring wheels started running
uint32_t scoringWheelsStartTime = 0;

// Remembers when the scoring macro started running
uint32_t scoringMacroStartTime = 0;

// Remembers when the scoring wheels started releasing
uint32_t scoringReleaseStartTime = 0;

// Remembers when the robot started driving forward
uint32_t macroDriveForwardStartTime = 0;

// Remembers the cascade position to move to before returning
double macroReturnLiftBumpTarget = 0;

// Remembers if the last macro movement timed out
bool scoringMacroTimedOut = false;

// Remembers which direction the macro was moving before timeout
bool scoringMacroTimedOutGoingUp = false;


// Start Macro

void startScoringMacro() {

    if (scoringMacroEnabled == false) {
        return;
    }

    // Start safety timer
    scoringMacroStartTime = pros::millis();

    // Retry the same movement if the last macro timed out
    if (scoringMacroTimedOut == true) {

        scoringMacroTimedOut = false;

        if (scoringMacroTimedOutGoingUp == true) {

            scoringPositionActive = true;

            // If cascade already cleared the starting position, continue both
            if (getLiftPosition() >= macroLiftStartScoringPosition) {
                scoringWheelsStartTime = pros::millis();
                scoringMacroState = MACRO_RAISING_BOTH;
            }
            else {
                scoringMacroState = MACRO_RAISING_LIFT;
            }
        }

        else {

            scoringPositionActive = false;

            // If cascade is already at the bottom, continue lowering scoring mech
            if (getLiftPosition() <= macroLiftBottomTolerance) {
                scoringMacroState = MACRO_RETURNING_SCORING;
            }
            else {
                scoringMacroState = MACRO_RETURNING_LIFT;
            }
        }

        return;
    }

    // If currently resting, move to scoring position
    if (scoringPositionActive == false) {

        scoringPositionActive = true;
        scoringMacroState = MACRO_RAISING_LIFT;
    }

    // If currently in scoring position, return everything
    else {

        scoringPositionActive = false;

        // Start scoring wheel release timer
        scoringReleaseStartTime = pros::millis();

        // Set cascade bump target from its current position
        macroReturnLiftBumpTarget =
            getLiftPosition() + macroReturnLiftBumpAmount;

        scoringMacroState = MACRO_RELEASING;
    }
}


// Update Macro

void updateScoringMacro() {

    // Nothing to do
    if (scoringMacroState == MACRO_IDLE) {
        return;
    }

    // Stop everything if the macro has been running for too long
    if (pros::millis() - scoringMacroStartTime >= macroSafetyTimeout) {

        if (
            scoringMacroState == MACRO_RAISING_LIFT
            ||
            scoringMacroState == MACRO_RAISING_BOTH
        ) {
            scoringMacroTimedOutGoingUp = true;
        }
        else {
            scoringMacroTimedOutGoingUp = false;
        }

        scoringMacroTimedOut = true;

        liftStop();
        scoringMechStop();
        scoringWheelsStop();
        macroLeftDriveMotors.move(0);
        macroRightDriveMotors.move(0);

        scoringMacroState = MACRO_IDLE;
        return;
    }


    // Step 1: Raise Cascade

    if (scoringMacroState == MACRO_RAISING_LIFT) {

        liftUp();
        scoringMechStop();

        // Cascade has cleared enough for scoring mech to start
        if (getLiftPosition() >= macroLiftStartScoringPosition) {
            scoringWheelsStartTime = pros::millis();
            scoringMacroState = MACRO_RAISING_BOTH;
        }
    }


    // Step 2: Raise Both

    else if (scoringMacroState == MACRO_RAISING_BOTH) {

        // Keep moving cascade until target
        if (getLiftPosition() < macroLiftTargetPosition) {
            liftUp();
        }
        else {
            liftStop();
        }

        // Move scoring mechanism until target
        if (getScoringMechPosition() < macroScoringTargetPosition) {
            scoringMechUp();
        }
        else {
            scoringMechStop();
        }

        // Keep scoring wheels accepting for the adjustable amount of time
        if (pros::millis() - scoringWheelsStartTime < macroScoringWheelsTime) {
            scoringWheelsAccept();
        }
        else {
            scoringWheelsStop();
        }

        // Both reached their targets
        if (
            getLiftPosition() >=
                macroLiftTargetPosition - macroPositionTolerance
            &&
            getScoringMechPosition() >=
                macroScoringTargetPosition - macroPositionTolerance
        ) {
            liftStop();
            scoringMechStop();
            scoringWheelsStop();

            scoringMacroState = MACRO_IDLE;
        }
    }


    // Release Scoring Wheels Before Returning

    else if (scoringMacroState == MACRO_RELEASING) {

        liftStop();
        scoringMechStop();

        if (
            pros::millis() - scoringReleaseStartTime
            <
            macroScoringReleaseTime
        ) {
            scoringWheelsReject();
        }
        else {
            scoringWheelsStop();

            scoringMacroState = MACRO_RETURN_LIFT_BUMP;
        }
    }


    // Raise Cascade a Little Before Returning

    else if (scoringMacroState == MACRO_RETURN_LIFT_BUMP) {

        scoringWheelsStop();
        scoringMechStop();

        if (
            getLiftPosition() <
            macroReturnLiftBumpTarget - macroPositionTolerance
        ) {
            liftUp();
        }
        else {
            liftStop();

            // Start forward drive timer
            macroDriveForwardStartTime = pros::millis();

            scoringMacroState = MACRO_DRIVING_FORWARD;
        }
    }


    // Drive Robot Forward Before Returning Cascade

    else if (scoringMacroState == MACRO_DRIVING_FORWARD) {

        liftStop();
        scoringMechStop();
        scoringWheelsStop();

        if (
            pros::millis() - macroDriveForwardStartTime
            <
            macroDriveForwardTime
        ) {
            macroLeftDriveMotors.move(macroDriveForwardSpeed);
            macroRightDriveMotors.move(macroDriveForwardSpeed);
        }
        else {
            macroLeftDriveMotors.move(0);
            macroRightDriveMotors.move(0);

            // Restart safety timer for the normal return sequence
            scoringMacroStartTime = pros::millis();

            scoringMacroState = MACRO_RETURNING_LIFT;
        }
    }


    // Return Cascade to Rest

    else if (scoringMacroState == MACRO_RETURNING_LIFT) {

        scoringWheelsStop();

        // Return cascade all the way to 0
        if (getLiftPosition() > macroLiftBottomTolerance) {
            liftDown();
            scoringMechStop();
        }
        else {
            liftStop();
            scoringMechStop();

            scoringMacroState = MACRO_RETURNING_SCORING;
        }
    }


    // Return Scoring Mechanism to Rest

    else if (scoringMacroState == MACRO_RETURNING_SCORING) {

        scoringWheelsStop();
        liftStop();

        // Return scoring mechanism to 0
        if (getScoringMechPosition() > macroPositionTolerance) {
            scoringMechDown();
        }
        else {
            scoringMechStop();
        }

        // Both are back at resting position
        if (
            getLiftPosition() <= macroLiftBottomTolerance
            &&
            getScoringMechPosition() <= macroPositionTolerance
        ) {
            liftStop();
            scoringMechStop();

            scoringMacroState = MACRO_IDLE;
        }
    }
}


// Macro Statuses

bool isScoringMacroRunning() {
    return scoringMacroState != MACRO_IDLE;
}

bool isScoringPositionActive() {
    return scoringPositionActive;
}