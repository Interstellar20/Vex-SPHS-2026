// gives file access to the lift and main files
# include "lift.h"
# include "main.h"
# include "opcontrol.h"
# include "robotConfigs.h"
# include "intake.h"

// two motors for the lift
pros::MotorGroup liftMotors({10, -16}, pros::MotorGears::green);

// speed of the lift
int liftSpeed = 127;

// LIFT POSITIONS

// lowest possible cascade position / match load position
double matchLoadPosition = 0;

// normal intake/resting position
// CHANGE THIS NUMBER after testing
double normalLiftPosition = 880;

// slows down when this close to a target
double liftSlowdownDistance = 150;

// how close the lift can be to count as at the target
double liftPositionTolerance = 15;

// slower speed used near a target
int liftSlowSpeed = 45;

// false = normal position
// true = match load position
bool matchLoadActive = false;

// true while automatically moving between normal and match load
bool matchLoadMoving = false;

// motors for lift always hold until a button is pressed to change it (idk if needed)
void liftSetup() {
    liftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    resetLiftPosition();
}

// three lift commands
void liftUp() {
    matchLoadMoving = false;
    matchLoadActive = false;
    liftMotors.move(liftSpeed);
}

void liftDown() {
    matchLoadMoving = false;
    matchLoadActive = false;

    // stop the lift from trying to go below 0
    if (getLiftPosition() <= matchLoadPosition) {
        liftMotors.move(0);
    }
    // slow down when getting close to 0
    else if (getLiftPosition() <= matchLoadPosition + liftSlowdownDistance) {
        liftMotors.move(-liftSlowSpeed);
    }
    else {
        liftMotors.move(-liftSpeed);
    }
}

// stops the lift
void liftStop() {
    liftMotors.move(0);
}

// smoothly moves the lift to a position without bouncing around it
bool moveLiftToPosition(double targetPosition) {
    double error = targetPosition - getLiftPosition();
    double distance = error;

    if (distance < 0) {
        distance = -distance;
    }

    // close enough to target
    if (distance <= liftPositionTolerance) {
        liftMotors.move(0);
        return true;
    }

    // moving up
    if (error > 0) {
        if (distance <= liftSlowdownDistance) {
            liftMotors.move(liftSlowSpeed);
        }
        else {
            liftMotors.move(liftSpeed);
        }
    }

    // moving down
    else {
        // never intentionally drive below 0
        if (getLiftPosition() <= matchLoadPosition) {
            liftMotors.move(0);
            return true;
        }

        if (distance <= liftSlowdownDistance) {
            liftMotors.move(-liftSlowSpeed);
        }
        else {
            liftMotors.move(-liftSpeed);
        }
    }

    return false;
}

// switches between match load position and normal intake position
void toggleMatchLoad() {

    // go to match load position
    if (matchLoadActive == false) {
        matchLoadActive = true;
    }

    // go back to normal intake position
    else {
        matchLoadActive = false;
    }

    matchLoadMoving = true;
}

// checks if automatic match load movement is still happening
bool isMatchLoadMoving() {

    if (matchLoadMoving == false) {
        return false;
    }

    double targetPosition;

    // figure out which position the cascade is currently moving to
    if (matchLoadActive == true) {
        targetPosition = matchLoadPosition;
    }
    else {
        targetPosition = normalLiftPosition;
    }

    // smoothly move toward the target
    if (moveLiftToPosition(targetPosition)) {
        matchLoadMoving = false;
    }

    return matchLoadMoving;
}

// gets the current lift motor position
double getLiftPosition() {
    return liftMotors.get_position(0);
}

// gets the normal intake/resting position
double getNormalLiftPosition() {
    return normalLiftPosition;
}

// gets the match load position
double getMatchLoadPosition() {
    return matchLoadPosition;
}

// makes the current lift position 0
void resetLiftPosition() {
    liftMotors.tare_position();

    matchLoadActive = true;
    matchLoadMoving = false;
}