#include "scoringMacro.h"
#include "lift.h"
#include "scoring.h"

// Scoring Macro Settings

// Turn entire macro on/off
bool scoringMacroEnabled = true;

// Cascade position where scoring mech automatically starts
double macroAutoScoringStartPosition = 560;

// Final scoring mechanism position
double macroScoringTargetPosition = 785;

// How long the scoring wheels accept after cascade reaches starting position
int macroScoringWheelsTime = 3000;

// How close the motors need to be to count as finished
double macroPositionTolerance = 25;

// How close the cascade needs to be to 0 to count as fully down
double macroLiftBottomTolerance = 10;

// How long the scoring wheels release before returning
int macroScoringReleaseTime = 500;

// How much farther the cascade moves up before returning
double macroReturnLiftBumpAmount = 260;

// How fast the robot drives forward before returning
int macroDriveForwardSpeed = 50;

// How long the robot drives forward before returning
int macroDriveForwardTime = 480;

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
    MACRO_RELEASING,
    MACRO_RETURN_LIFT_BUMP,
    MACRO_DRIVING_FORWARD,
    MACRO_RETURNING_LIFT,
    MACRO_RETURNING_SCORING
};

ScoringMacroState scoringMacroState = MACRO_IDLE;

// false = cascade is below automatic scoring position
// true = cascade has reached automatic scoring position
bool scoringPositionActive = false;

// Remembers if scoring mech is still automatically moving to scoring position
bool scoringMechAutoMoving = false;

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


// Start Macro

void startScoringMacro() {

    if (scoringMacroEnabled == false) {
        return;
    }

    // Only start return macro if cascade is up
    if (getLiftPosition() < macroAutoScoringStartPosition) {
        return;
    }

    // Stop automatic scoring mech movement
    scoringMechAutoMoving = false;

    // Start safety timer
    scoringMacroStartTime = pros::millis();

    // Start scoring wheel release timer
    scoringReleaseStartTime = pros::millis();

    // Set cascade bump target from its current position
    macroReturnLiftBumpTarget =
        getLiftPosition() + macroReturnLiftBumpAmount;

    scoringMacroTimedOut = false;
    scoringPositionActive = false;

    scoringMacroState = MACRO_RELEASING;
}


// Update Macro

void updateScoringMacro() {

    // Automatically prepare scoring mechanism when cascade reaches position
    if (
        scoringMacroEnabled == true
        &&
        scoringMacroState == MACRO_IDLE
    ) {

        // Cascade crossed into automatic scoring position
        if (
            getLiftPosition() >= macroAutoScoringStartPosition
            &&
            scoringPositionActive == false
        ) {
            scoringPositionActive = true;
            scoringMechAutoMoving = true;
            scoringWheelsStartTime = pros::millis();
        }

        // Cascade is below automatic scoring position again
        if (
            getLiftPosition() < macroAutoScoringStartPosition
            &&
            scoringPositionActive == true
        ) {
            scoringPositionActive = false;
            scoringMechAutoMoving = false;
        }

        // Automatically move scoring mechanism to scoring position one time
        if (scoringMechAutoMoving == true) {

            if (
                getScoringMechPosition()
                <
                macroScoringTargetPosition - macroPositionTolerance
            ) {
                scoringMechUp();
            }
            else {
                // Stop automatic movement once target is reached
                // HOLD brake keeps scoring mech in place
                scoringMechStop();
                scoringMechAutoMoving = false;
            }
        }

        // Run scoring wheels for adjustable amount of time
        if (
            scoringPositionActive == true
            &&
            pros::millis() - scoringWheelsStartTime
            <
            macroScoringWheelsTime
        ) {
            scoringWheelsAccept();
        }
        else {
            scoringWheelsStop();
        }

        return;
    }


    // Nothing to do
    if (scoringMacroState == MACRO_IDLE) {
        return;
    }


    // Stop everything if the macro has been running for too long
    if (pros::millis() - scoringMacroStartTime >= macroSafetyTimeout) {

        scoringMacroTimedOut = true;

        liftStop();
        scoringMechStop();
        scoringWheelsStop();
        macroLeftDriveMotors.move(0);
        macroRightDriveMotors.move(0);

        scoringMacroState = MACRO_IDLE;
        return;
    }


    // Release Scoring Wheels Before Returning

    if (scoringMacroState == MACRO_RELEASING) {

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
            getLiftPosition()
            <
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

            scoringPositionActive = false;
            scoringMechAutoMoving = false;
            scoringMacroTimedOut = false;

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