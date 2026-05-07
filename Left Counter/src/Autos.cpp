#include "main.h"
#include "ExternalSystems.hpp"
// ── Routine implementations ───────────────────────────────────────────────────

void autoSkills() {
    chassis.setPose(0, 0, 0);
    Descore.set_value(1);
    intakeFront.move(127);
    chassis.moveToPoint(0,14.6,490,{.maxSpeed=127,.minSpeed=60,.earlyExitRange=6});
    intakeFront.move(127);
    chassis.swingToHeading(-32,lemlib::DriveSide::LEFT,500,{.maxSpeed=127,.minSpeed=50,.earlyExitRange=2});
    intakeFront.move(127);
    chassis.moveToPoint(-31,40.75,1080,{.maxSpeed=110,.minSpeed=20,.earlyExitRange=2});
    chassis.waitUntil(22.25);
    Matchload.set_value(1);
    chassis.moveToPoint(-19,8,1300,{.forwards=false,.maxSpeed=115});
    pros::delay(80);
    Matchload.set_value(0);
    chassis.turnToHeading(88,450);
    intakeTop.move(25);
    pros::delay(50);
    intakeTop.move(0);
    chassis.moveToPoint(-36,10,600,{.forwards=false});
    chassis.turnToHeading(178,470);
    chassis.moveToPoint(-31,-16.5,1550,{.maxSpeed=55});
    intakeFront.move(127);
    Matchload.set_value(1);
    chassis.moveToPoint(-31,-2.6,800,{.forwards=false,.minSpeed=80,.earlyExitRange=3});
    chassis.turnToHeading(212,470,{.minSpeed=80,.earlyExitRange=3});
    chassis.moveToPoint(-18.5,13.5,1200,{.forwards=false,.maxSpeed=110,.minSpeed=80,.earlyExitRange=3});
    chassis.turnToHeading(177,450);
    chassis.moveToPoint(-20,40,1300,{false});
    chassis.waitUntilDone();
    Descore.set_value(0);
    pros::delay(100);
    chassis.moveToPoint(-20,17.5,1000,{.minSpeed=80,.earlyExitRange=3});
    chassis.waitUntil(10);
    intake.move(-80);
    chassis.turnToHeading(215,500,{.minSpeed=80,.earlyExitRange=3});
    intake.move(0);
    Descore.set_value(1);
    chassis.moveToPoint(-31,-1.5,1200,{.maxSpeed=110,.minSpeed=80,.earlyExitRange=3});
    chassis.turnToHeading(179,425);
    chassis.moveToPoint(-32.5,25.5,1500,{.forwards=false});
    chassis.waitUntil(15.5);
    intake.move(127);
    chassis.waitUntilDone();
    intake.move(0);
    chassis.moveToPoint(-32.75,-2.4,800);
    chassis.turnToHeading(222,450);
    chassis.moveToPoint(6,37.75,1500,{.forwards=false,.maxSpeed=100});
    intake.move(-100);
    pros::delay(150);
    intake.move(0);
    chassis.waitUntil(24);
    middleGoal.set_value(1);
    intakeFront.move(127);
    chassis.waitUntilDone();
}

void autoRedClose()  { /* TODO */ }
void autoRedFar()    { /* TODO */ }
void autoBlueClose() { /* TODO */ }
void autoBlueFar()   { /* TODO */ }