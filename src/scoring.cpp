#include "scoring.h"
#include "main.h"
#include "lift.h"
#include "motionView.h"

// Scoring mechanism up/down motor
pros::Motor scoringMech(8);

// Scoring flex wheel motor
pros::Motor scoringWheelMotor(9);

int scoringMechSpeed = 80;
int scoringWheelsSpeed = 45;

// Automatic scoring settings
double scoringAutoUpHeight = 1200;
double scoringAutoDownHeight = 950;
double scoring90Position = 890;
double scoringFoldedPosition = 0;
double scoringAutoTolerance = 25;

// Remembers the last lift position
double previousLiftPosition = 0;

// Automatic movement
bool scoringAutoMovingUp = false;
bool scoringAutoMovingDown = false;

// false = flex wheels move one direction
// true = flex wheels move the other direction
bool scoringDirection = true;

void scoringSetup() {
    scoringMech.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    resetScoringMechPosition();
    previousLiftPosition = getLiftPosition();
}

void scoringMechUp() {
    scoringMech.move(scoringMechSpeed);
}

void scoringMechDown() {
    scoringMech.move(-scoringMechSpeed);
}

void scoringMechStop() {
    scoringMech.move(0);
}

// Used by Eva's profile
void scoringWheels() {
    if (scoringDirection == false) {
        scoringWheelMotor.move(scoringWheelsSpeed);
    }
    else {
        scoringWheelMotor.move(-scoringWheelsSpeed);
    }
}

// Used by Ansh's profile
void scoringWheelsAccept() {
    scoringWheelMotor.move(-scoringWheelsSpeed);
}

// Used by Ansh's profile
void scoringWheelsReject() {
    scoringWheelMotor.move(scoringWheelsSpeed);
}

void scoringWheelsStop() {
    scoringWheelMotor.move(0);
}

void changeScoringDirection() {
    scoringDirection = !scoringDirection;
}

// Checks if the lift crossed an automatic scoring height
void checkScoringAutoTrigger() {
    double currentLiftPosition = getLiftPosition();

    // Unfold when the lift crosses the upper height
    if (previousLiftPosition < scoringAutoUpHeight &&
        currentLiftPosition >= scoringAutoUpHeight) {
        scoringAutoMovingUp = true;
        scoringAutoMovingDown = false;
    }

    // Fold when the lift crosses the lower height
    if (previousLiftPosition > scoringAutoDownHeight &&
        currentLiftPosition <= scoringAutoDownHeight) {
        scoringAutoMovingUp = false;
        scoringAutoMovingDown = true;
    }

    previousLiftPosition = currentLiftPosition;
}

// Runs the automatic scoring movement
bool updateScoringAuto() {
    checkScoringAutoTrigger();

    // Move scoring mech to 90 degrees
    if (scoringAutoMovingUp) {
        if (getScoringMechPosition() < scoring90Position - scoringAutoTolerance) {
            scoringMechUp();
            return true;
        }

        scoringMechStop();
        scoringAutoMovingUp = false;
        return false;
    }

    // Fold scoring mech
    if (scoringAutoMovingDown) {
        if (getScoringMechPosition() > scoringFoldedPosition + scoringAutoTolerance) {
            scoringMechDown();
            return true;
        }

        scoringMechStop();
        scoringAutoMovingDown = false;
        return false;
    }

    return false;
}

// Cancels automatic scoring movement
void cancelScoringAuto() {
    scoringAutoMovingUp = false;
    scoringAutoMovingDown = false;
}

// Resets automatic scoring detection
void syncScoringAuto() {
    previousLiftPosition = getLiftPosition();
    scoringAutoMovingUp = false;
    scoringAutoMovingDown = false;
}

// Checks if scoring mech is automatically moving up
bool isScoringAutoMovingUp() {
    return scoringAutoMovingUp;
}

// Checks if automatic scoring movement is happening
bool isScoringAutoRunning() {
    return scoringAutoMovingUp || scoringAutoMovingDown;
}

// Gets the current scoring mechanism position
double getScoringMechPosition() {
    return scoringMech.get_position();
}

// Makes the current scoring mechanism position 0
void resetScoringMechPosition() {
    scoringMech.tare_position();
}