#include "scoringMacro.h"
#include "main.h"
#include "robotConfigs.h"
#include "lift.h"
#include "scoring.h"

bool scoringMacroEnabled = true;

// Lift tolerance
double macroLiftTolerance = 20;

// Scoring fold tolerance
double macroScoringFoldTolerance = 50;

// Release scoring wheels
int macroScoringReleaseTime = 500;

// Small lift bump
int macroLiftBumpTime = 200;

// Drive forward
int macroDriveForwardSpeed = 50;
int macroDriveForwardTime = 300;

// Maximum macro time
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
uint32_t macroDriveForwardStartTime = 0;

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
    if (!scoringMacroEnabled) {
        return;
    }

    if (scoringMacroState != MACRO_IDLE) {
        return;
    }

    // Stop match load preset
    cancelMatchLoad();

    // Stop normal scoring auto movement
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
    if (scoringMacroState == MACRO_IDLE) {
        return;
    }

    // Safety stop
    if (
        pros::millis() - scoringMacroStartTime >=
        static_cast<uint32_t>(macroSafetyTimeout)
    ) {
        finishScoringMacro();
        return;
    }

    // Release scoring wheels
    if (scoringMacroState == MACRO_RELEASING) {
        liftStop();
        scoringMechStop();
        stopMacroDrive();

        if (
            pros::millis() - scoringReleaseStartTime <
            static_cast<uint32_t>(macroScoringReleaseTime)
        ) {
            scoringWheelsReject();
        }
        else {
            scoringWheelsStop();

            macroLiftBumpStartTime = pros::millis();

            scoringMacroState = MACRO_LIFT_BUMP;
        }

        return;
    }

    // Small upward cascade bump
    if (scoringMacroState == MACRO_LIFT_BUMP) {
        scoringMechStop();
        scoringWheelsStop();
        stopMacroDrive();

        if (
            pros::millis() - macroLiftBumpStartTime <
            static_cast<uint32_t>(macroLiftBumpTime)
        ) {
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

        if (
            pros::millis() - macroDriveForwardStartTime <
            static_cast<uint32_t>(macroDriveForwardTime)
        ) {
            chassis.arcade(
                macroDriveForwardSpeed,
                0,
                true
            );
        }
        else {
            stopMacroDrive();

            scoringMacroState = MACRO_RETURNING_LIFT;
        }

        return;
    }

    // Return cascade to intake height
    if (scoringMacroState == MACRO_RETURNING_LIFT) {
        scoringMechStop();
        scoringWheelsStop();
        stopMacroDrive();

        double currentPosition = getLiftPosition();
        double intakePosition = getNormalLiftPosition();

        // Cascade is above intake
        if (
            currentPosition >
            intakePosition + macroLiftTolerance
        ) {
            liftDown();
            return;
        }

        // Cascade is below intake
        if (
            currentPosition <
            intakePosition - macroLiftTolerance
        ) {
            liftUp();
            return;
        }

        // Cascade reached intake
        liftStop();

        scoringMacroState = MACRO_RETURNING_SCORING;

        return;
    }

    // Fold scoring mechanism
    if (scoringMacroState == MACRO_RETURNING_SCORING) {
        liftStop();
        scoringWheelsStop();
        stopMacroDrive();

        double scoringPosition = getScoringMechPosition();

        // Only move downward toward folded
        if (scoringPosition > macroScoringFoldTolerance) {
            scoringMechDown();
            return;
        }

        // Do not reverse upward if it overshoots
        scoringMechStop();

        finishScoringMacro();

        return;
    }
}

bool isScoringMacroRunning() {
    return scoringMacroState != MACRO_IDLE;
}

bool isScoringPositionActive() {
    return false;
}