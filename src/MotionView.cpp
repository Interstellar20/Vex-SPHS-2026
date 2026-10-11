#include "main.h"
#include "robotConfigs.h"
#include "opcontrol.h"
#include "lift.h"
#include "intake.h"
#include "scoring.h"
#include "motionView.h"
#include "scoringMacro.h"

#include "mvlib/api.hpp"
#include "lemlib/api.hpp"
#include "mvlib/Optional/lemlib.hpp"

void initializeMotionView() {
    auto& logger = mvlib::Logger::getInstance();
    logger.setBuildDate(__DATE__);

    // Attach your odom 
    mvlib::setOdom(&chassis);
    // Needed for accurate speed telemetry
    logger.setRobot({
    .leftDrivetrain = &left_motor_group,
    .rightDrivetrain = &right_motor_group
    });

    logger.start();
}