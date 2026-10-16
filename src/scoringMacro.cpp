#include "scoringMacro.h"
#include "main.h"
#include "robotConfigs.h"
#include "lift.h"
#include "scoring.h"

bool scoringMacroEnabled = true;

// Macro settings
double macroLiftTolerance = 20;
double macroScoringFoldTolerance = 50;
int macroScoringReleaseTime = 500;
int macroLiftBumpTime = 1000;
int macroBumpStallCheckTime = 250;
double macroBumpMinimumMovement = 5;
int macroDriveForwardSpeed = 50;
int macroDriveForwardTime = 300;
int macroSafetyTimeout = 10000;

enum ScoringMacroState {
    MACRO_IDLE,
    MACRO_RELEASING,
    MACRO_LIFT_BUMP,
    MACRO_DRIVING_FORWARD,
    MACRO_RETURNING_LIFT,
    MACRO_RETURNING_SCORING
};

ScoringMacroState scoringMacroState = MACRO_IDLE;

uint32_t scoringMacroStartTime = 0;
uint32_t scoringReleaseStartTime = 0;
uint32_t macroLiftBumpStartTime = 0;
uint32_t macroBumpCheckTime = 0;
uint32_t macroDriveForwardStartTime = 0;
double macroBumpLastPosition = 0;

void stopMacroDrive() {
    chassis.arcade(0, 0, true);
}

void finishScoringMacro() {
    liftStop();
    scoringMechStop();
    scoringWheelsStop();
    stopMacroDrive();
    scoringMacroState = MACRO_IDLE;
    syncScoringAuto();
}

void startScoringMacro() {
    if (!scoringMacroEnabled || scoringMacroState != MACRO_IDLE) return;

    cancelMatchLoad();
    cancelScoringAuto();
    liftStop();
    scoringMechStop();
    scoringWheelsStop();
    stopMacroDrive();

    scoringMacroStartTime = pros::millis();
    scoringReleaseStartTime = pros::millis();
    scoringMacroState = MACRO_RELEASING;
}

void updateScoringMacro() {
    if (scoringMacroState == MACRO_IDLE) return;

    if (pros::millis() - scoringMacroStartTime >=
        static_cast<uint32_t>(macroSafetyTimeout)) {
        finishScoringMacro();
        return;
    }

    // Release scoring wheels
    if (scoringMacroState == MACRO_RELEASING) {
        liftStop();
        scoringMechStop();
        stopMacroDrive();

        if (pros::millis() - scoringReleaseStartTime <
            static_cast<uint32_t>(macroScoringReleaseTime)) {
            scoringWheelsReject();
        }
        else {
            scoringWheelsStop();
            macroLiftBumpStartTime = pros::millis();
            macroBumpCheckTime = pros::millis();
            macroBumpLastPosition = getLiftPosition();
            scoringMacroState = MACRO_LIFT_BUMP;
        }
        return;
    }

    // Bump upward for up to 1 second
    if (scoringMacroState == MACRO_LIFT_BUMP) {
        scoringMechStop();
        scoringWheelsStop();
        stopMacroDrive();

        bool bumpFinished = pros::millis() - macroLiftBumpStartTime >=
            static_cast<uint32_t>(macroLiftBumpTime);

        if (pros::millis() - macroBumpCheckTime >=
            static_cast<uint32_t>(macroBumpStallCheckTime)) {
            double movement = getLiftPosition() - macroBumpLastPosition;
            if (movement < macroBumpMinimumMovement) bumpFinished = true;
            macroBumpLastPosition = getLiftPosition();
            macroBumpCheckTime = pros::millis();
        }

        if (!bumpFinished) {
            liftUp();
        }
        else {
            liftStop();
            macroDriveForwardStartTime = pros::millis();
            scoringMacroState = MACRO_DRIVING_FORWARD;
        }
        return;
    }

    // Drive forward
    if (scoringMacroState == MACRO_DRIVING_FORWARD) {
        liftStop();
        scoringMechStop();
        scoringWheelsStop();

        if (pros::millis() - macroDriveForwardStartTime <
            static_cast<uint32_t>(macroDriveForwardTime)) {
            chassis.arcade(macroDriveForwardSpeed, 0, true);
        }
        else {
            stopMacroDrive();
            scoringMacroState = MACRO_RETURNING_LIFT;
        }
        return;
    }

    // Return cascade smoothly to intake
    if (scoringMacroState == MACRO_RETURNING_LIFT) {
        scoringMechStop();
        scoringWheelsStop();
        stopMacroDrive();

        if (moveLiftToPosition(getNormalLiftPosition())) {
            scoringMacroState = MACRO_RETURNING_SCORING;
        }
        return;
    }

    // Fold scoring mechanism
    if (scoringMacroState == MACRO_RETURNING_SCORING) {
        liftStop();
        scoringWheelsStop();
        stopMacroDrive();

        if (getScoringMechPosition() > macroScoringFoldTolerance) {
            scoringMechDown();
            return;
        }

        scoringMechStop();
        finishScoringMacro();
    }
}

bool isScoringMacroRunning() {
    return scoringMacroState != MACRO_IDLE;
}

bool isScoringPositionActive() {
    return false;
}