#include "scoring.h"
#include "main.h"
#include "lift.h"

// Scoring motors
pros::Motor scoringMech(8);
pros::Motor scoringWheelMotor(9);

int scoringMechSpeed = 80;
int scoringWheelsSpeed = 45;

// Automatic scoring settings
double scoringAutoUpHeight = 1200;
double scoringAutoDownHeight = 950;
double scoring90Position = 920;
double scoringFoldedPosition = 0;
double scoringAutoTolerance = 25;

double previousLiftPosition = 0;
bool scoringAutoMovingUp = false;
bool scoringAutoMovingDown = false;
bool scoringDirection = true;

void scoringSetup() {
    scoringMech.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    scoringMech.tare_position();
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

void scoringWheels() {
    scoringWheelMotor.move(scoringDirection ? -scoringWheelsSpeed : scoringWheelsSpeed);
}

void scoringWheelsAccept() {
    scoringWheelMotor.move(-scoringWheelsSpeed);
}

void scoringWheelsReject() {
    scoringWheelMotor.move(scoringWheelsSpeed);
}

void scoringWheelsStop() {
    scoringWheelMotor.move(0);
}

void changeScoringDirection() {
    scoringDirection = !scoringDirection;
}

void checkScoringAutoTrigger() {
    double currentLiftPosition = getLiftPosition();

    if (previousLiftPosition < scoringAutoUpHeight &&
        currentLiftPosition >= scoringAutoUpHeight) {
        scoringAutoMovingUp = true;
        scoringAutoMovingDown = false;
    }

    if (previousLiftPosition > scoringAutoDownHeight &&
        currentLiftPosition <= scoringAutoDownHeight) {
        scoringAutoMovingUp = false;
        scoringAutoMovingDown = true;
    }

    previousLiftPosition = currentLiftPosition;
}

bool updateScoringAuto() {
    checkScoringAutoTrigger();

    if (scoringAutoMovingUp) {
        if (getScoringMechPosition() < scoring90Position - scoringAutoTolerance) {
            scoringMechUp();
            return true;
        }
        scoringMechStop();
        scoringAutoMovingUp = false;
        return false;
    }

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

void cancelScoringAuto() {
    scoringAutoMovingUp = false;
    scoringAutoMovingDown = false;
}

void syncScoringAuto() {
    previousLiftPosition = getLiftPosition();
    scoringAutoMovingUp = false;
    scoringAutoMovingDown = false;
}

bool isScoringAutoMovingUp() {
    return scoringAutoMovingUp;
}

bool isScoringAutoRunning() {
    return scoringAutoMovingUp || scoringAutoMovingDown;
}

double getScoringMechPosition() {
    return scoringMech.get_position();
}

void resetScoringMechPosition() {
    scoringMech.tare_position();
}