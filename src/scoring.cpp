#include "scoring.h"
#include "main.h"
#include "lift.h"

// Scoring mechanism motor
pros::Motor scoringMech(8);

// Scoring flex wheel motor
pros::Motor scoringWheelMotor(9);

int scoringMechSpeed = 80;
int scoringWheelsSpeed = 45;

// Lift heights that control automatic scoring
double scoringAutoUpHeight = 450;
double scoringAutoDownHeight = 350;

// Scoring mechanism positions
double scoring90Position = 890;
double scoringFoldedPosition = 0;

// Position tolerances
double scoringAutoTolerance = 25;
double scoringFoldedTolerance = 50;

// Last lift position
double previousLiftPosition = 0;

// Automatic movement
bool scoringAutoMovingUp = false;
bool scoringAutoMovingDown = false;

// Flex wheel direction
bool scoringDirection = true;

void scoringSetup() {
    scoringMech.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    scoringWheelMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // Start robot with scoring mech fully folded
    resetScoringMechPosition();

    previousLiftPosition = getLiftPosition();

    scoringAutoMovingUp = false;
    scoringAutoMovingDown = false;

    scoringMechStop();
    scoringWheelsStop();
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
    if (scoringDirection == false) {
        scoringWheelMotor.move(scoringWheelsSpeed);
    }
    else {
        scoringWheelMotor.move(-scoringWheelsSpeed);
    }
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

    // Lift crossed upward past 450
    if (
        previousLiftPosition < scoringAutoUpHeight &&
        currentLiftPosition >= scoringAutoUpHeight
    ) {
        scoringAutoMovingUp = true;
        scoringAutoMovingDown = false;
    }

    // Lift crossed downward past 350
    if (
        previousLiftPosition > scoringAutoDownHeight &&
        currentLiftPosition <= scoringAutoDownHeight
    ) {
        scoringAutoMovingUp = false;
        scoringAutoMovingDown = true;
    }

    previousLiftPosition = currentLiftPosition;
}

bool updateScoringAuto() {
    checkScoringAutoTrigger();

    // Automatically open scoring mech
    if (scoringAutoMovingUp) {
        double position = getScoringMechPosition();

        if (position < scoring90Position - scoringAutoTolerance) {
            scoringMechUp();
            return true;
        }

        scoringMechStop();

        scoringAutoMovingUp = false;

        return false;
    }

    // Automatically fold scoring mech
    if (scoringAutoMovingDown) {
        double position = getScoringMechPosition();

        // Only move toward folded
        // Do not reverse upward if it overshoots a little
        if (position > scoringFoldedPosition + scoringFoldedTolerance) {
            scoringMechDown();
            return true;
        }

        scoringMechStop();

        scoringAutoMovingDown = false;

        return false;
    }

    // Hold wherever manual button was released
    scoringMechStop();

    return false;
}

void cancelScoringAuto() {
    scoringAutoMovingUp = false;
    scoringAutoMovingDown = false;

    previousLiftPosition = getLiftPosition();
}

void syncScoringAuto() {
    previousLiftPosition = getLiftPosition();

    scoringAutoMovingUp = false;
    scoringAutoMovingDown = false;

    scoringMechStop();
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