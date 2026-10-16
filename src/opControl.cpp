#include "main.h"
#include "opcontrol.h"
#include "robotConfigs.h"
#include "lift.h"
#include "intake.h"
#include "scoring.h"
#include "scoringMacro.h"

// Driver Curve Settings
lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);

// Ansh
double anshSteerExpo = 1.019;
double anshThrottleExpo = 1.019;

// Eva
double evaSteerExpo = 1.019;
double evaThrottleExpo = 1.019;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

DriverMode driverMode = PRACTICE;
bool practiceSkills = false;

void updateDriverCurves(double &steerExpo, double &throttleExpo) {
    bool yHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

    if (yHeld) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            steerExpo -= 0.001;
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            steerExpo += 0.001;
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            throttleExpo -= 0.001;
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            throttleExpo += 0.001;
        }
    }

    if (steerExpo < 1.000) {
        steerExpo = 1.000;
    }

    if (throttleExpo < 1.000) {
        throttleExpo = 1.000;
    }

    controller.print(
        2,
        0,
        "S:%.3f T:%.3f",
        steerExpo,
        throttleExpo
    );
}

// Ansh Driver
void competitionDriver() {
    updateDriverCurves(anshSteerExpo, anshThrottleExpo);

    bool yHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

    // DOWN starts scoring macro
    if (
        !yHeld &&
        controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)
    ) {
        startScoringMacro();
    }

    // Drive Controls
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    lemlib::ExpoDriveCurve throttleCurve(3, 10, anshThrottleExpo);
    lemlib::ExpoDriveCurve steerCurve(3, 10, anshSteerExpo);

    int curvedThrottle = throttleCurve.curve(leftY);
    int curvedSteer = steerCurve.curve(rightX);

    if (!isScoringMacroRunning()) {
        chassis.arcade(curvedThrottle, curvedSteer, true);
    }

    // Lift Controls
    if (!isScoringMacroRunning()) {
        // B is the only way below intake height
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            toggleMatchLoad();
        }

        // X raises cascade
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            cancelMatchLoad();
            liftUp();
        }

        // A lowers cascade only to intake height
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            cancelMatchLoad();

            if (getLiftPosition() > getNormalLiftPosition() + 20) {
                liftDown();
            }
            else {
                liftStop();
            }
        }

        else if (!isMatchLoadMoving()) {
            liftStop();
        }

        updateMatchLoad();
    }

    // Intake Controls
    bool intakeAcceptHeld =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

    if (intakeAcceptHeld) {
        intakeAccept();
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        intakeReject();
    }
    else {
        intakeStop();
    }

    // Scoring Mechanism Controls
    if (!isScoringMacroRunning()) {
        // Hold R1 to move up
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            syncScoringAuto();
            scoringMechUp();
        }

        // Hold L1 to move down
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            syncScoringAuto();
            scoringMechDown();
        }

        // Release button and hold position
        else {
            if (!updateScoringAuto()) {
                scoringMechStop();
            }
        }
    }

    // Scoring Wheel Controls
    if (intakeAcceptHeld) {
        scoringWheelsAccept();
    }
    else if (
        !isScoringMacroRunning() &&
        isScoringAutoMovingUp()
    ) {
        scoringWheelsAccept();
    }
    else if (!yHeld) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            scoringWheelsAccept();
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            scoringWheelsReject();
        }
        else {
            scoringWheelsStop();
        }
    }
    else {
        scoringWheelsStop();
    }

    updateScoringMacro();

    printf(
        "Lift: %.2f | Intake: %.2f | Scoring: %.2f | Macro: %d\n",
        getLiftPosition(),
        getNormalLiftPosition(),
        getScoringMechPosition(),
        isScoringMacroRunning()
    );

    pros::delay(25);
}

// Eva Driver
void skillsDriver() {
    updateDriverCurves(evaSteerExpo, evaThrottleExpo);

    bool yHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

    // DOWN starts scoring macro
    if (
        !yHeld &&
        controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)
    ) {
        startScoringMacro();
    }

    // Drive Controls
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    lemlib::ExpoDriveCurve throttleCurve(3, 10, evaThrottleExpo);
    lemlib::ExpoDriveCurve steerCurve(3, 10, evaSteerExpo);

    int curvedThrottle = throttleCurve.curve(leftY);
    int curvedSteer = steerCurve.curve(rightX);

    if (!isScoringMacroRunning()) {
        chassis.arcade(curvedThrottle, curvedSteer, true);
    }

    // Lift Controls
    if (!isScoringMacroRunning()) {
        // A is the only way below intake height
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            toggleMatchLoad();
        }

        // L1 raises cascade
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            cancelMatchLoad();
            liftUp();
        }

        // L2 lowers cascade only to intake height
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            cancelMatchLoad();

            if (getLiftPosition() > getNormalLiftPosition() + 20) {
                liftDown();
            }
            else {
                liftStop();
            }
        }

        else if (!isMatchLoadMoving()) {
            liftStop();
        }

        updateMatchLoad();
    }

    // Intake Controls
    bool intakeAcceptHeld =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);

    if (intakeAcceptHeld) {
        intakeAccept();
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        intakeReject();
    }
    else {
        intakeStop();
    }

    // Scoring Mechanism Controls
    if (!isScoringMacroRunning()) {
        // Hold X to move up
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            syncScoringAuto();
            scoringMechUp();
        }

        // Hold B to move down
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            syncScoringAuto();
            scoringMechDown();
        }

        // Release button and hold position
        else {
            if (!updateScoringAuto()) {
                scoringMechStop();
            }
        }
    }

    // Scoring Wheel Controls
    if (intakeAcceptHeld) {
        scoringWheelsAccept();
    }
    else if (
        !isScoringMacroRunning() &&
        isScoringAutoMovingUp()
    ) {
        scoringWheelsAccept();
    }
    else if (!yHeld) {
        if (
            controller.get_digital_new_press(
                pros::E_CONTROLLER_DIGITAL_UP
            )
        ) {
            changeScoringDirection();
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            scoringWheels();
        }
        else {
            scoringWheelsStop();
        }
    }
    else {
        scoringWheelsStop();
    }

    updateScoringMacro();

    printf(
        "Lift: %.2f | Intake: %.2f | Scoring: %.2f | Macro: %d\n",
        getLiftPosition(),
        getNormalLiftPosition(),
        getScoringMechPosition(),
        isScoringMacroRunning()
    );

    pros::delay(25);
}

// Practice Driver
void practiceDriver() {
    // X + A selects Ansh
    if (
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
        controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)
    ) {
        practiceSkills = false;
        controller.rumble(".-");
        pros::delay(500);
    }

    // X + B selects Eva
    if (
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
        controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)
    ) {
        practiceSkills = true;
        controller.rumble("-...");
        pros::delay(500);
    }

    if (practiceSkills == false) {
        competitionDriver();
    }
    else {
        skillsDriver();
    }
}