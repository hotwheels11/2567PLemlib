#include "main.h"
#include "Autos.hpp" // IWYU pragma: keep
#include "ExternalSystems.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"


void initialize() {
    //pros::lcd::initialize();
	chassis.calibrate();// make sure chassis is ready
    pros::Task coordDisplayTask(Coords);
}
/*
inline void intakeManagementSystem(void*) {
    while (true) {

        if (defaultState == 0) {
            // ===== MANUAL CONTROL =====
            if (master.get_digital(DIGITAL_R1)) {
                intakeFront.move(127);
            } 
            else if (master.get_digital(DIGITAL_R2)) {
                intake.move(-127);
            } 
            else if (master.get_digital(DIGITAL_L1)) {
                intake.move(127);
            } 
            else if (master.get_digital(DIGITAL_L2)) {
				middleGoalState = 1;
                intake.move(-127);
            } 
            else {
				middleGoalState = 0;
                intake.move(0);
            }
        }

        else {
            // ===== AUTO COLOR SORT =====
            intakeColorSort();
        }

        pros::delay(10); // short, non-blocking delay
    }
}
*/
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
void competition_initialize() {
	//ts::selector::get()->display();
}

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
    chassis.setPose(0, 0, 0);
    Descore.set_value(1);
    intakeFront.move(127);
    chassis.moveToPoint(0,15.5,500,{.maxSpeed=127,.minSpeed=20,.earlyExitRange=2}); //Drives towards 4 stack
    chassis.swingToHeading(-38,lemlib::DriveSide::LEFT,450,{.maxSpeed=127}); //Swings into 4 stack
    chassis.moveToPoint(-14,23.7,900); //Drives into 4 stack
    chassis.waitUntil(10);
    intakeFront.move(0);
    chassis.turnToHeading(-132,500); //Turns to Middle Goal
    intake.move(-35);
    chassis.moveToPoint(1,27.8,800,{.forwards=false}); //Backs into middle goal
    Matchload.set_value(1);
    intake.move(0);
    chassis.waitUntilDone();
    middleGoal.set_value(1); //Starts the scoring to middle goal
    pros::delay(50);
    intakeFront.move(127);
    pros::delay(700);
    chassis.moveToPoint(-37,-11,1300); //Drives to matchload #1
    intake.move(-127);
    pros::delay(100);
    intake.move(127);
    chassis.turnToHeading(-178,500); //Turns to matchload #1
    intake.move(0);
    intakeFront.move(127);
    middleGoal.set_value(0);
    chassis.moveToPoint(-34.9,-30,1800,{.maxSpeed=80}); //Goes into matchload #1
    chassis.moveToPoint(-33.5,-11,800,{.forwards=false});//Backs out from Matchload #1
    chassis.waitUntilDone();
    Matchload.set_value(0);
    chassis.turnToHeading(-45,500); //Turns to Ally
    chassis.moveToPoint(-49,19,1000); //Drives before ally
    intakeFront.move(0);
    chassis.turnToHeading(-1,500); //Turns into ally
    chassis.moveToPoint(-46,95.1,2000,{.maxSpeed=100}); //Drives down ally
    chassis.turnToHeading(-88,600); //turns to long goal
    chassis.moveToPoint(-39.25,89,900,{.forwards=false}); //Back up perpendicular to long goal
    chassis.turnToHeading(-4,700); //Turns to long goal #2
    chassis.moveToPoint(-33.3,70,900,{.forwards=false}); //Backs into long goal
    chassis.waitUntil(18);
    intake.move(127);
    pros::delay(400);
    intake.move(-127);
    pros::delay(100);
    intake.move(127);
    pros::delay(1300); //Scores in long goal
    Matchload.set_value(1);
    chassis.moveToPoint(-33,123,3500,{.maxSpeed=50}); //Drives into matchload #2
    intakeTop.move(0);
    chassis.moveToPoint(-33,75,1000,{.forwards=false,.maxSpeed=90}); //backs to long goal #1 Second time
    chassis.waitUntil(2);
    intake.move(-127);
    pros::delay(50);
    intake.move(0);
    chassis.waitUntilDone();
    intake.move(127);
    pros::delay(1000);
    pros::delay(1200);
    intake.move(0);
    chassis.moveToPoint(-33,98.7,800); 
    Matchload.set_value(0);
    chassis.turnToHeading(63,500); //TUrns to Blue Park
    chassis.moveToPoint(-4,112,1000);
    chassis.swingToHeading(87,lemlib::DriveSide::RIGHT,500); //Swings to blue park
    intakeFront.move(127);
    chassis.moveToPoint(44,115,5000,{.maxSpeed=127,.minSpeed=100});
    Matchload.set_value(1); 
    chassis.waitUntil(12);
    Matchload.set_value(0);
    chassis.waitUntil(40);
    Matchload.set_value(1);
    chassis.swingToHeading(179,lemlib::DriveSide::RIGHT, 1000); //Swing in prep to distance reset
    chassis.waitUntilDone();
    bool success = autoDistanceReset(chassis); //Distance Reset
    pros::delay(200);
    chassis.turnToHeading(235,500); //turn past 4 stack
    Matchload.set_value(0);
    chassis.moveToPoint(-10,113,1000);//Move infront of 4 stack
    chassis.turnToHeading(130,500); //Turns to 4 stack
    intakeFront.move(127); //Starts intake
    chassis.moveToPoint(18,100,1000); //Drives into 4 stack
    chassis.waitUntil(9);
    intakeFront.move(0);
    Matchload.set_value(1);
    chassis.turnToHeading(45,500); //Faces the back of middle goal
    intake.move(-127);
    chassis.moveToPoint(0,92,1000,{.forwards=false}); //Backs into middle goal #2
    intake.move(0);
    chassis.waitUntilDone();
    middleGoal.set_value(1); //Allows the scoring to middle goal
    pros::delay(50);
    intakeFront.move(80);
    pros::delay(1300);
    intakeFront.move(0);
    chassis.moveToPoint(46,130,1000); //Drive infront of matchload #3
    chassis.waitUntil(5);
    intakeFront.move(127);
    chassis.turnToHeading(6,500); //Turns to matchload #3
    chassis.moveToPoint(43,150,3500,{.maxSpeed=55}); //Drives into matchload #3
    chassis.waitUntil(2);
    middleGoal.set_value(0);
    chassis.waitUntilDone();
    chassis.moveToPoint(42,135,1000,{.forwards=false,.maxSpeed=127});//Backs out from Matchload #3
    chassis.waitUntilDone();
    Matchload.set_value(0);
    chassis.turnToHeading(150,500); //Turns to ally
    chassis.moveToPoint(55,104.6,1000); //Drives infront of ally
    chassis.turnToHeading(183,500); //Turns into ally
    chassis.moveToPoint(46.8,24.7,2000,{.maxSpeed=100}); //Drives down ally

}  

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
bool middleGoalState = false;

bool middleGoalManual = false;
inline int middleGoalDescoreState = 0;
void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    Descore.set_value(1);
    color_sensor.set_led_pwm(100);

    while (true) {
        // ===== DRIVE =====
        int leftY  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        chassis.tank(leftY, rightY);

        // ===== DESCORE TOGGLE (B) =====
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            descore++;
        }
        Descore.set_value(descore % 2 == 1);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            middleGoalDescoreState++;
        }
        middleGoalDescore.set_value(middleGoalDescoreState % 2 == 1);
        // ===== MATCHLOAD TOGGLE (Y) =====
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            MatchloadMech++;
        }
        Matchload.set_value(MatchloadMech % 2 == 0);

        // ===== MIDDLE GOAL MANUAL TOGGLE (DOWN) =====
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            middleGoalManual = !middleGoalManual;
        }

        // ===== MIDDLE GOAL COMMAND =====
        bool middleGoalCommand =
            middleGoalManual || master.get_digital(DIGITAL_L2);
            middleGoal.set_value(middleGoalCommand);

        // ===== INTAKE CONTROL =====
        if (master.get_digital(DIGITAL_R1)) {
            intakeFront.move(127);
        }
        else if (master.get_digital(DIGITAL_R2)) {
            intake.move(-127);
        }
        else if (master.get_digital(DIGITAL_L1)) {
            intake.move(127);
        }
        else if (master.get_digital(DIGITAL_L2)) {
            intakeFront.move(80);
        }
        else {
            intake.move(0);
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            bool success = autoDistanceReset(chassis);
            master.rumble(success ? "." : "..");
        }

        pros::delay(20);
    }
}