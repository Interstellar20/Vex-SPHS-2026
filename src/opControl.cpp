#include "main.h"
#include "opcontrol.h"
#include "robotConfigs.h"
#include "lift.h"
#include "intake.h"
#include "scoring.h"
#include "scoringMacro.h"


// Driver Curve Settings

lemlib::ExpoDriveCurve throttle_curve(
    3,
    10,
    1.019
);

lemlib::ExpoDriveCurve steer_curve(
    3,
    10,
    1.019
);


// Competition / Ansh
double anshSteerExpo = 1.019;
double anshThrottleExpo = 1.019;

// Skills / Eva
double evaSteerExpo = 1.019;
double evaThrottleExpo = 1.019;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

DriverMode driverMode = PRACTICE; // default driver mode is practice

bool practiceSkills = false; // Allows practice driver to switch between competition and skills driver profiles WITHOUT changing the Current Mode


// Driver Curve Adjustment

void updateDriverCurves(double &steerExpo, double &throttleExpo) {

    bool yHeld = controller.get_digital(
        pros::E_CONTROLLER_DIGITAL_Y
    );

    if (yHeld) {

        // Y + LEFT = decrease steering sensitivity
        if (controller.get_digital_new_press(
            pros::E_CONTROLLER_DIGITAL_LEFT
        )) {
            steerExpo -= 0.001;
        }

        // Y + RIGHT = increase steering sensitivity
        if (controller.get_digital_new_press(
            pros::E_CONTROLLER_DIGITAL_RIGHT
        )) {
            steerExpo += 0.001;
        }

        // Y + DOWN = decrease throttle sensitivity
        if (controller.get_digital_new_press(
            pros::E_CONTROLLER_DIGITAL_DOWN
        )) {
            throttleExpo -= 0.001;
        }

        // Y + UP = increase throttle sensitivity
        if (controller.get_digital_new_press(
            pros::E_CONTROLLER_DIGITAL_UP
        )) {
            throttleExpo += 0.001;
        }
    }

    // Don't allow values below 1.000
    if (steerExpo < 1.000) {
        steerExpo = 1.000;
    }

    if (throttleExpo < 1.000) {
        throttleExpo = 1.000;
    }

    // Show both current values on the bottom controller line
    controller.print(
        2,
        0,
        "S:%.3f T:%.3f",
        steerExpo,
        throttleExpo
    );
}


// Competition driver profile

void competitionDriver() {

    // Adjust Ansh's steering/throttle curves
    updateDriverCurves(
        anshSteerExpo,
        anshThrottleExpo
    );

    bool yHeld = controller.get_digital(
        pros::E_CONTROLLER_DIGITAL_Y
    );

    // DOWN arrow starts/reverses scoring macro
    if (
        !yHeld &&
        controller.get_digital_new_press(
            pros::E_CONTROLLER_DIGITAL_DOWN
        )
    ) {
        startScoringMacro();
    }

    // Drive Controls
    int leftY = controller.get_analog(
        pros::E_CONTROLLER_ANALOG_LEFT_Y
    );

    int rightX = controller.get_analog(
        pros::E_CONTROLLER_ANALOG_RIGHT_X
    );

    // Create curves using Ansh's current settings
    lemlib::ExpoDriveCurve throttleCurve(
        3,
        10,
        anshThrottleExpo
    );

    lemlib::ExpoDriveCurve steerCurve(
        3,
        10,
        anshSteerExpo
    );

    // Apply adjustable curves
    int curvedThrottle = throttleCurve.curve(leftY);
    int curvedSteer = steerCurve.curve(rightX);

    // Arcade Drive
    // true disables LemLib's built-in chassis curves because
    // we already applied adjustable curves above
    chassis.arcade(
        curvedThrottle,
        curvedSteer,
        true
    );

    // Lift Controls
    if (!isScoringMacroRunning()) {

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            liftUp();
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            liftDown();
        }
        else {
            liftStop();
        }
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

    // Scoring Mechanism Up/Down Controls
    if (!isScoringMacroRunning()) {

        // Each button press moves the scoring mechanism one 10 degree snap
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            scoringMechSnapUp();
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            scoringMechSnapDown();
        }
    }

    // Scoring Flex Wheel Controls
    // Intake accept always makes the scoring wheels accept
    if (intakeAcceptHeld) {
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

    // Run scoring macro
    updateScoringMacro();

    printf(
        "Lift: %.2f | Scoring: %.2f\n",
        getLiftPosition(),
        getScoringMechPosition()
    );

    // Delay to prevent overloading the controller
    pros::delay(25);
}


// Skills driver profile

void skillsDriver() {

    // Adjust Eva's steering/throttle curves
    updateDriverCurves(
        evaSteerExpo,
        evaThrottleExpo
    );

    bool yHeld = controller.get_digital(
        pros::E_CONTROLLER_DIGITAL_Y
    );

    // DOWN arrow starts/reverses scoring macro
    if (
        !yHeld &&
        controller.get_digital_new_press(
            pros::E_CONTROLLER_DIGITAL_DOWN
        )
    ) {
        startScoringMacro();
    }

    // Drivetrain Controls
    int leftY = controller.get_analog(
        pros::E_CONTROLLER_ANALOG_LEFT_Y
    );

    int rightX = controller.get_analog(
        pros::E_CONTROLLER_ANALOG_RIGHT_X
    );

    // Create curves using Eva's current settings
    lemlib::ExpoDriveCurve throttleCurve(
        3,
        10,
        evaThrottleExpo
    );

    lemlib::ExpoDriveCurve steerCurve(
        3,
        10,
        evaSteerExpo
    );

    // Apply adjustable curves
    int curvedThrottle = throttleCurve.curve(leftY);
    int curvedSteer = steerCurve.curve(rightX);

    // Arcade Drive
    // true disables LemLib's built-in chassis curves because
    // we already applied our adjustable curves above
    chassis.arcade(
        curvedThrottle,
        curvedSteer,
        true
    );

    // Lift Controls
    if (!isScoringMacroRunning()) {

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            liftUp();
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            liftDown();
        }
        else {
            liftStop();
        }
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

    // Scoring Mechanism Up/Down Controls
    if (!isScoringMacroRunning()) {

        // Each button press moves the scoring mechanism one 10 degree snap
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            scoringMechSnapUp();
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            scoringMechSnapDown();
        }
    }

    // Scoring Flex Wheel Controls
    // Intake accept always makes the scoring wheels accept
    if (intakeAcceptHeld) {
        scoringWheelsAccept();
    }
    else if (!yHeld) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
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

    // Run scoring macro
    updateScoringMacro();

    printf(
        "Lift: %.2f | Scoring: %.2f\n",
        getLiftPosition(),
        getScoringMechPosition()
    );

    // Delay to prevent overloading the controller
    pros::delay(25);
}


// Practice driver profile

void practiceDriver() {

    // X+A = Competition Driver
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)
     && controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {

        practiceSkills = false;

        controller.rumble(".-");
        pros::delay(500);
    }

    // X+B = Skills Driver
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)
     && controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {

        practiceSkills = true;

        controller.rumble("-...");
        pros::delay(500);
    }

    // Run whichever driver profile is selected
    if (practiceSkills == false) {
        competitionDriver();
    }
    else {
        skillsDriver();
    }
}