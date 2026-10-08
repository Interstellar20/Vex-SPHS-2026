#include "scoring.h"
#include "main.h"

// Scoring mechanism up/down motor
pros::Motor scoringMech(8);

// Scoring flex wheel motor
pros::Motor scoringWheelMotor(9);

int scoringMechSpeed = 80;
int scoringWheelsSpeed = 65;

// How many scoring motor encoder degrees equal one 10 degree snap
double scoringMechSnapAmount = 89;

// false = flex wheels move one direction
// true = flex wheels move the other direction
bool scoringDirection = true;

void scoringSetup() {
    scoringMech.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    resetScoringMechPosition();
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


// Manual Scoring Mechanism Angle Snapping

void scoringMechSnapUp() {

    double currentPosition = getScoringMechPosition();

    double targetPosition =
        currentPosition + scoringMechSnapAmount;

    scoringMech.move_absolute(
        targetPosition,
        scoringMechSpeed
    );

    printf(
        "Scoring Snap UP | Current: %.2f | Target: %.2f\n",
        currentPosition,
        targetPosition
    );
}

void scoringMechSnapDown() {

    double currentPosition = getScoringMechPosition();

    double targetPosition =
        currentPosition - scoringMechSnapAmount;

    scoringMech.move_absolute(
        targetPosition,
        scoringMechSpeed
    );

    printf(
        "Scoring Snap DOWN | Current: %.2f | Target: %.2f\n",
        currentPosition,
        targetPosition
    );
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

// gets the current scoring mechanism position
double getScoringMechPosition() {
    return scoringMech.get_position();
}

// makes the current scoring mechanism position 0
void resetScoringMechPosition() {
    scoringMech.tare_position();
}