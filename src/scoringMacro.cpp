#include "scoringMacro.h"
#include "main.h"
#include "lift.h"
#include "scoring.h"

// Scoring Macro Settings

// Turn entire macro on/off
bool scoringMacroEnabled = true;

// How close the motors need to be to count as finished
double macroPositionTolerance = 25;

// How long the scoring wheels release before returning
int macroScoringReleaseTime = 500;

// How much farther the cascade moves up before returning
double macroReturnLiftBumpAmount = 270;

// How fast the robot drives forward before returning
int macroDriveForwardSpeed = 50;

// How long the robot drives forward before returning
int macroDriveForwardTime = 300;

// Maximum amount of time the scoring macro can run
int macroSafetyTimeout = 3000;

// Drive Motors
pros::MotorGroup macroLeftDriveMotors({-15, -12}, pros::MotorGears::blue);
pros::MotorGroup macroRightDriveMotors({19, 11}, pros::MotorGears::blue);

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

// Remembers when the scoring macro started running
uint32_t scoringMacroStartTime = 0;

// Remembers when the scoring wheels started releasing
uint32_t scoringReleaseStartTime = 0;

// Remembers when the robot started driving forward
uint32_t macroDriveForwardStartTime = 0;

// Remembers the cascade position to move to before returning
double macroReturnLiftBumpTarget = 0;

// Start Macro
void startScoringMacro() {
    if (scoringMacroEnabled == false) {
        return;
    }

    // Do not restart the macro while it is running
    if (scoringMacroState != MACRO_IDLE) {
        return;
    }

    // Stop automatic scoring movement
    cancelScoringAuto();

    // Start timers
    scoringMacroStartTime = pros::millis();
    scoringReleaseStartTime = pros::millis();

    // Set cascade bump target
    macroReturnLiftBumpTarget = getLiftPosition() + macroReturnLiftBumpAmount;

    scoringMacroState = MACRO_RELEASING;
}

// Update Macro
void updateScoringMacro() {
    if (scoringMacroState == MACRO_IDLE) {
        return;
    }

    // Stop everything if the macro takes too long
    if (pros::millis() - scoringMacroStartTime >= macroSafetyTimeout) {
        liftStop();
        scoringMechStop();
        scoringWheelsStop();
        macroLeftDriveMotors.move(0);
        macroRightDriveMotors.move(0);
        scoringMacroState = MACRO_IDLE;
        syncScoringAuto();
        return;
    }

    // Release scoring wheels
    if (scoringMacroState == MACRO_RELEASING) {
        liftStop();
        scoringMechStop();

        if (pros::millis() - scoringReleaseStartTime < macroScoringReleaseTime) {
            scoringWheelsReject();
        }
        else {
            scoringWheelsStop();
            scoringMacroState = MACRO_RETURN_LIFT_BUMP;
        }
    }

    // Raise cascade a little
    else if (scoringMacroState == MACRO_RETURN_LIFT_BUMP) {
        scoringWheelsStop();
        scoringMechStop();

        if (getLiftPosition() < macroReturnLiftBumpTarget - macroPositionTolerance) {
            liftUp();
        }
        else {
            liftStop();
            macroDriveForwardStartTime = pros::millis();
            scoringMacroState = MACRO_DRIVING_FORWARD;
        }
    }

    // Drive robot forward
    else if (scoringMacroState == MACRO_DRIVING_FORWARD) {
        liftStop();
        scoringMechStop();
        scoringWheelsStop();

        if (pros::millis() - macroDriveForwardStartTime < macroDriveForwardTime) {
            macroLeftDriveMotors.move(macroDriveForwardSpeed);
            macroRightDriveMotors.move(macroDriveForwardSpeed);
        }
        else {
            macroLeftDriveMotors.move(0);
            macroRightDriveMotors.move(0);
            scoringMacroStartTime = pros::millis();
            scoringMacroState = MACRO_RETURNING_LIFT;
        }
    }

    // Return cascade to normal intake position
    else if (scoringMacroState == MACRO_RETURNING_LIFT) {
        scoringWheelsStop();
        scoringMechStop();

        if (moveLiftToPosition(getNormalLiftPosition())) {
            scoringMacroState = MACRO_RETURNING_SCORING;
        }
    }

    // Return scoring mechanism to rest
    else if (scoringMacroState == MACRO_RETURNING_SCORING) {
        scoringWheelsStop();
        liftStop();

        if (getScoringMechPosition() > macroPositionTolerance) {
            scoringMechDown();
        }
        else {
            scoringMechStop();
        }

        // Finish the macro
        if (getLiftPosition() <= getNormalLiftPosition() + macroPositionTolerance &&
            getLiftPosition() >= getNormalLiftPosition() - macroPositionTolerance &&
            getScoringMechPosition() <= macroPositionTolerance) {
            liftStop();
            scoringMechStop();
            scoringMacroState = MACRO_IDLE;
            syncScoringAuto();
        }
    }
}

// Macro Statuses
bool isScoringMacroRunning() {
    return scoringMacroState != MACRO_IDLE;
}

bool isScoringPositionActive() {
    return false;
}