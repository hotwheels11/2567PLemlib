#include "main.h"

namespace Auton {
    
void redLeft() {
    // Example autonomous routine
    Robot::chassis->moveToPoint(48.0f, 48.0f, 3000);
    Robot::chassis->waitUntil(10.0f);
    Systems::intake.move(127);
    Robot::chassis->waitUntilDone();
    Systems::intake.move(0);
    
    Robot::chassis->turnToHeading(90.0f, 2000);
    Robot::chassis->waitUntilDone();
    
    Robot::chassis->moveToPoint(60.0f, 30.0f, 4000, {.maxSpeed = 80.0});
    Robot::chassis->waitUntilDone();
}

void redRight() {
    Robot::chassis->moveToPoint(24.0f, 24.0f, 3000, {.forwards = false});
    Robot::chassis->waitUntilDone();
}

void blueLeft() {
    Robot::chassis->moveToPoint(-20.0f, 40.0f, 3000, {.earlyExitRange = 5.0});
    Robot::chassis->waitUntilDone();
}

void blueRight() {
    Robot::chassis->turnToHeading(180.0f, 2000, {.maxSpeed = 60.0});
    Robot::chassis->waitUntilDone();
}

void skills() {
    Robot::chassis->moveToPoint(0.0f, 0.0f, 4000);
    Robot::chassis->waitUntil(20.0f);
    Robot::chassis->cancelMotion();
}

} // namespace Auton