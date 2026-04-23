#include "main.h"
#include "Autos.hpp"
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
    chassis.moveToPoint(0,14.6,490,{.maxSpeed=127,.minSpeed=60,.earlyExitRange=6}); //Drives towards 3 stack
    intakeFront.move(127);
    chassis.swingToHeading(-32,lemlib::DriveSide::LEFT,500,{.maxSpeed=127,.minSpeed=50,.earlyExitRange=2}); //Swings into 3 stack
    intakeFront.move(127);
    chassis.moveToPoint(-31,40.75,1080,{.maxSpeed=110,.minSpeed=20,.earlyExitRange=2}); //Collects 2 blocks under long
    chassis.waitUntil(22.25);
    Matchload.set_value(1);
    chassis.moveToPoint(-19,8,1300,{.forwards=false,.maxSpeed=115}); //reverses from 2 stack
    pros::delay(80);
    Matchload.set_value(0);
    chassis.turnToHeading(88,450);
    intakeTop.move(25);
    pros::delay(50);
    intakeTop.move(0);
    chassis.moveToPoint(-36,10,600,{.forwards=false});
    chassis.turnToHeading(178,470); //Turns to matchloader
    chassis.moveToPoint(-31,-16.5,1550,{.maxSpeed=55}); //Goes into matchload
    intakeFront.move(127);
    Matchload.set_value(1);
    chassis.moveToPoint(-31,-2.6,800,{.forwards=false,.minSpeed=80,.earlyExitRange=3});//backs out of matchloader
    chassis.turnToHeading(212,470,{.minSpeed=80,.earlyExitRange=3});
    chassis.moveToPoint(-18.5,13.5,1200,{.forwards=false,.maxSpeed=110,.minSpeed=80,.earlyExitRange=3}); //Goes towards long goal
    chassis.turnToHeading(177,450); //Turns parallel to long goal
    chassis.moveToPoint(-20,40,1300,{false}); //Backs to the edge of long goal
    chassis.waitUntilDone();
    Descore.set_value(0); //Puts wing down
    pros::delay(100);
    chassis.moveToPoint(-20,17.5,1000,{.minSpeed=80,.earlyExitRange=3}); //Moves forward to descore long goal
    chassis.waitUntil(10);
    intake.move(-80);
    chassis.turnToHeading(215,500,{.minSpeed=80,.earlyExitRange=3}); //Turns
    intake.move(0);
    Descore.set_value(1); //Puts wing up to avoid collisions
    chassis.moveToPoint(-31,-1.5,1200,{.maxSpeed=110,.minSpeed=80,.earlyExitRange=3}); //Gets before long goal scoring pose
    chassis.turnToHeading(179,425); //Turns to score in long goal
    chassis.moveToPoint(-32.5,25.5,1500,{.forwards=false}); //Backs into long goal
    chassis.waitUntil(15.5);
    intake.move(127);
    chassis.waitUntilDone();
    intake.move(0);
    chassis.moveToPoint(-32.75,-2.4,800); //Drives out of long goal
    chassis.turnToHeading(222,450); //Turns to upper Middle
    chassis.moveToPoint(6,37.75,1500,{.forwards=false,.maxSpeed=100}); //Backs into Upper Middle
    intake.move(-100);
    pros::delay(150);
    intake.move(0);
    chassis.waitUntil(24);
    middleGoal.set_value(1);
    intakeFront.move(127);
    chassis.waitUntilDone();
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
            intakeFront.move(110);
        }
        else {
            intake.move(0);
        }

        pros::delay(20);
    }
}