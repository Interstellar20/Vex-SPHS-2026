#include "lift.h"
#include "main.h"

// Cascade motors
pros::Motor liftMotor1(10);
pros::Motor liftMotor2(-16);

int liftSpeed = 127;

// Measured lift positions
double matchLoadPosition = 0;
double normalLiftPosition = 310;
double maxLiftPosition = 4130;

// How close the lift needs to be
double liftPositionTolerance = 20;

// Match load preset
bool matchLoadMoving = false;
double matchLoadTarget = 0;

void liftSetup() {
    liftMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    liftMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // Start robot with cascade fully down
    liftMotor1.tare_position();
    liftMotor2.tare_position();

    matchLoadMoving = false;

    liftStop();
}

void liftUp() {
    // Stop at the measured maximum height
    if (getLiftPosition() >= maxLiftPosition) {
        liftStop();
        return;
    }

    liftMotor1.move(liftSpeed);
    liftMotor2.move(liftSpeed);
}

void liftDown() {
    liftMotor1.move(-liftSpeed);
    liftMotor2.move(-liftSpeed);
}

void liftStop() {
    liftMotor1.move(0);
    liftMotor2.move(0);
}

double getLiftPosition() {
    // Port 10 is used for the lift position
    return liftMotor1.get_position();
}

double getNormalLiftPosition() {
    return normalLiftPosition;
}

double getMatchLoadPosition() {
    return matchLoadPosition;
}

double getMaxLiftPosition() {
    return maxLiftPosition;
}

bool moveLiftToPosition(double targetPosition) {
    double currentPosition = getLiftPosition();

    if (currentPosition > targetPosition + liftPositionTolerance) {
        liftDown();
        return false;
    }

    if (currentPosition < targetPosition - liftPositionTolerance) {
        liftUp();
        return false;
    }

    liftStop();

    return true;
}

void toggleMatchLoad() {
    double currentPosition = getLiftPosition();

    // If at intake height or above,
    // move all the way down
    if (currentPosition >= normalLiftPosition - liftPositionTolerance) {
        matchLoadTarget = matchLoadPosition;
    }

    // If below intake height,
    // return to intake height
    else {
        matchLoadTarget = normalLiftPosition;
    }

    matchLoadMoving = true;
}

void updateMatchLoad() {
    if (!matchLoadMoving) {
        return;
    }

    if (moveLiftToPosition(matchLoadTarget)) {
        matchLoadMoving = false;
    }
}

void cancelMatchLoad() {
    matchLoadMoving = false;
}

bool isMatchLoadMoving() {
    return matchLoadMoving;
}