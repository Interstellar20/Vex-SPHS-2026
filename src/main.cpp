#include "main.h"
#include "robotConfigs.h"
#include "opcontrol.h"
#include "lift.h"
#include "intake.h"
#include "scoring.h"
#include "motionView.h"
#include "scoringMacro.h"
#include "auton.h"

// Driver Profile Selector

void updateScreenTask() {
    while (true) {
        pros::lcd::print(0, "X: %f", chassis.getPose().x);
        pros::lcd::print(1, "Y: %f", chassis.getPose().y);
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
        pros::delay(50);
    }
}

void selectDriverMode() {
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_MEDIUM, 20, 30, "DRIVER MODE");

    // Draw Competition, Skills, and Practice buttons

    // Competition button
    pros::screen::fill_rect(40, 50, 440, 100);
    pros::screen::print(pros::E_TEXT_MEDIUM, 150, 65, "COMPETITION");

    // Skills button
    pros::screen::fill_rect(40, 115, 440, 165);
    pros::screen::print(pros::E_TEXT_MEDIUM, 190, 130, "SKILLS");

    // Practice button
    pros::screen::fill_rect(40, 180, 440, 230);
    pros::screen::print(pros::E_TEXT_MEDIUM, 185, 195, "PRACTICE");

    while (true) {
        auto status = pros::screen::touch_status();

        if (status.touch_status == pros::E_TOUCH_PRESSED) {
            if (status.x >= 40 && status.x <= 440 &&
                status.y >= 50 && status.y <= 100) {
                driverMode = COMPETITION;
                break;
            }

            if (status.x >= 40 && status.x <= 440 &&
                status.y >= 115 && status.y <= 165) {
                driverMode = SKILLS;
                break;
            }

            if (status.x >= 40 && status.x <= 440 &&
                status.y >= 180 && status.y <= 230) {
                driverMode = PRACTICE;
                break;
            }
        }

        pros::delay(20);
    }

    pros::screen::erase();

    if (driverMode == COMPETITION) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 20, 30, "COMPETITION SELECTED");
    } else if (driverMode == SKILLS) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 20, 30, "SKILLS SELECTED");
    } else {
        pros::screen::print(pros::E_TEXT_MEDIUM, 20, 30, "PRACTICE SELECTED");
    }

    pros::delay(1000);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize the LCD screen

    chassis.calibrate(); //calibrates the IMU and sets the initial position to 0,0,0

    initializeMotionView(); // initializes the motion view
    selectDriverMode(); // allows the user to select the driver mode (competition, skills, or practice)
    liftSetup(); // sets the lift motors to hold position when stopped
    intakeSetup(); // sets the intake motors to hold position when stopped
    scoringSetup(); // sets the scoring mechanism motor to hold position when stopped
    pros::Task screen_task(updateScreenTask);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    testAuton();
}

void opcontrol() {
	//Continous loop for driver to control motors
	while (true) {

        if (driverMode == COMPETITION) {
            competitionDriver();
        }
        else if (driverMode == SKILLS) {
            skillsDriver();
        }
        else {
            practiceDriver();
        }

        pros::delay(10);     
	}
}