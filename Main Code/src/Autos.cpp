#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "ExternalSystems.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"

void line_Up_Auto(){
    chassis.moveToPoint(0,14,2000,{.maxSpeed = 40});
}
void line_Up_Solo(){
    chassis.setPose(-62.469,-16.707,90);
	chassis.moveToPose(-43,-16.3,90,1000,{.maxSpeed=30});
}

void WP_Auto(){
    chassis.setPose(-43,-16.3,90);
	intake.move(127);
	chassis.moveToPoint(-33.5,-16.3,800);
	chassis.waitUntilDone();
	chassis.swingToHeading(134, lemlib::DriveSide::RIGHT, 200,{},false);
	chassis.waitUntil(2.7);
	intakemiddle.move(50);
	Matchload.set_value(1);
	chassis.moveToPoint(-16.7,-23,800);
	chassis.waitUntilDone();
	intakemiddle.move(0);
	chassis.turnToHeading(49, 600);
	chassis.waitUntilDone();
	Matchload.set_value(0);
	chassis.moveToPoint(-12.3, -19, 800);
	chassis.waitUntilDone();
	intake.move(-127);
	pros::delay(750);
	intake.move(127);
	chassis.moveToPoint(-17, -27.6, 800,{.forwards=false});
	chassis.waitUntilDone();
	chassis.turnToHeading(0, 400);
	chassis.waitUntilDone();
	chassis.moveToPoint(-15, 18, 1200,{.maxSpeed=100});
	chassis.waitUntil(38);
	Matchload.set_value(1);
	chassis.waitUntilDone();
	chassis.turnToHeading(-44, 400);
	chassis.waitUntilDone();
	chassis.moveToPoint(-1, 6,800,{.forwards=false});
	chassis.waitUntil(2);
	Matchload.set_value(0);
	chassis.waitUntilDone();
	intake.move(-127);
	pros::delay(400);
	intake.move(127);
	intakelate.move(-127);
	pros::delay(800);
	intakelate.move(127);
	chassis.turnToHeading(-38, 700,{},false);
	chassis.moveToPoint(-38, 46, 1200,{.maxSpeed=90});
	chassis.waitUntilDone();
	chassis.turnToHeading(-90, 600);
	chassis.waitUntilDone();
	Matchload.set_value(1);
	pros::delay(50);
	chassis.moveToPoint(-70, 46, 1500,{.maxSpeed=60});
	chassis.moveToPoint(-14, 47, 3000,{.forwards=false,.maxSpeed=90});
	chassis.waitUntil(4);
	intake.move(0);
	pros::delay(500);
	Hood.set_value(1);
	intake.move(127);
}

void right_Side() {
    chassis.setPose(0, 0, 0);
	Descore.set_value(1);
    chassis.moveToPoint(0, 13, 1000);
    intake.move(127);
    chassis.moveToPoint(9.0, 28, 1000);//get first three balls
    chassis.waitUntil(3);
    Matchload.set_value(1); //Activates to hold balls in place
    chassis.waitUntilDone();
    Matchload.set_value(0); //Deactivates
    pros::delay(1000);
    chassis.moveToPoint(27, 47, 2000);
    pros::delay(500);
    Matchload.set_value(1); //Activates to hold balls in place
    chassis.moveToPoint(9,29,1000,{.forwards=false,.maxSpeed=60}); //moves back to three ball area
    chassis.turnToHeading(-43,500); //moves to middle goal
    Matchload.set_value(0); //Deactivates
    chassis.moveToPoint(1.5, 39.5, 800); //drives into middle goal
    intake.move(-127); //outtakes balls into middle goal
    pros::delay(1350);
    intake.move(127);//starts intake fwd
    intakeFront.move(40);
    chassis.moveToPoint(37, 6.7, 1300,{.forwards=false,.maxSpeed=80}); //backs away from middle goal
    chassis.turnToHeading(-180,1000);
    Matchload.set_value(1); //Activates
    chassis.moveToPoint(34,-7,1500,{.maxSpeed=60}); //Drives into Matchload Area
    chassis.waitUntilDone();
    intakeFront.move(127);
    chassis.moveToPoint(36, 29, 1000,{.forwards=false,.maxSpeed=70}); //backs out of matchload area
    chassis.waitUntilDone();
    Hood.set_value(1);
    intake.move(127);
    Matchload.set_value(0); //Deactivates
    Descore.set_value(0);
}

void left_Side() {
    chassis.setPose(0, 0, 0);
	Descore.set_value(1);
    chassis.moveToPoint(0, 13, 700);
    intake.move(127);
    chassis.moveToPoint(-9.0, 28, 700);//get first three balls
    chassis.waitUntil(3.5);
    Matchload.set_value(1); //Activates to hold balls in place
    chassis.waitUntilDone();
    Matchload.set_value(0); //Deactivates
    pros::delay(400);
	chassis.turnToHeading(-49, 800);
    chassis.moveToPoint(-24, 45, 1500);
    pros::delay(300);
    Matchload.set_value(1); //Activates to hold balls in place
    chassis.moveToPoint(-7,26.4,1000,{.forwards=false,.maxSpeed=70}); //moves back to three ball area
    chassis.turnToHeading(-133, 1000); //turn to middle goal
    Matchload.set_value(0); //Deactivates
    chassis.moveToPoint(8, 44, 800,{.forwards=false}); //drives into middle goal
	intake.move(-127);
	intakeFront.move(0);
	pros::delay(920);
    intake.move(127); //outtakes balls into middle goal
	intakelate.move(-85);
    pros::delay(1530);
	intakelate.move(127);
    chassis.moveToPoint(-30, 9.9, 1300,{.maxSpeed=90}); //backs away from middle goal
    chassis.turnToHeading(-180,1000,{.maxSpeed=90});
    Matchload.set_value(1); //Activates
    chassis.moveToPoint(-33.2,-12,1600,{.maxSpeed=65}); //Drives into Matchload Area
    chassis.waitUntilDone();
    intakeFront.move(127);
    chassis.moveToPoint(-33, 38, 2000,{.forwards=false,.maxSpeed=60}); //backs out of matchload area
    chassis.waitUntilDone();
    Hood.set_value(1);
    intake.move(127);
    Matchload.set_value(0); //Deactivates
    Descore.set_value(0);
}

void calculatePoseAuto() {
    Matchload.set_value(1); //Activates to hold balls in place
    pros::delay(500);
    calculates_robot_position(); // saves automatically in savedPose
    pros::delay(1000);
    Matchload.set_value(0); //Deactivates
}

void Skills_Auto(){
    setChassisToSavedPose(); // sets the chassis to previously saved position
    Descore.set_value(1);
    chassis.moveToPoint(46,43,1000,{.maxSpeed=80});
    chassis.waitUntilDone();
    chassis.turnToHeading(90,900);
    chassis.waitUntilDone();
    pros::delay(200);
    Matchload.set_value(1);
    chassis.moveToPoint(60,50,2800,{.maxSpeed=60});
    intake.move(127);
    chassis.waitUntilDone();
    Matchload.set_value(0);
    chassis.moveToPoint(49.1,50,800,{.forwards=false});
    chassis.waitUntilDone();
    chassis.turnToHeading(-50,900);
    chassis.waitUntilDone();
    chassis.moveToPoint(30,62.5,1000,{.maxSpeed=100});
    chassis.waitUntil(12);
    chassis.cancelMotion();
    chassis.moveToPoint(-25,61.3,2000,{.maxSpeed=100});
    chassis.waitUntil(60);
    chassis.cancelMotion();
    chassis.moveToPoint(-43,5,1500,{.maxSpeed=80});
    chassis.waitUntilDone();
    chassis.turnToHeading(-90,900);
    chassis.waitUntilDone();
    chassis.moveToPoint(-25,49,2200,{.forwards=false,.maxSpeed=60});
    chassis.waitUntil(8);
    Hood.set_value(1);
    chassis.waitUntilDone();
    Hood.set_value(0); 
    Matchload.set_value(1);
    chassis.moveToPoint(-63,48,2000,{.maxSpeed=60});
    chassis.waitUntilDone();
    chassis.moveToPoint(-25,48,2000,{.forwards=false,.maxSpeed=60});
    chassis.waitUntil(15);
    Hood.set_value(1);
    chassis.waitUntilDone();

} 