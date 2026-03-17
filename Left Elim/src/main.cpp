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
    chassis.moveToPoint(0,14.6,490,{.maxSpeed=127,.minSpeed=20,.earlyExitRange=2}); //Drives towards 3 stack
    intakeFront.move(127);
    chassis.swingToHeading(-32,lemlib::DriveSide::LEFT,500,{.maxSpeed=127,.minSpeed=50,.earlyExitRange=2}); //Swings into 3 stack
    intakeFront.move(127);
    chassis.moveToPoint(-31.5,40,1080,{.maxSpeed=110,.minSpeed=20,.earlyExitRange=2}); //Collects 2 blocks under long
    chassis.waitUntil(21.7);
    Matchload.set_value(1);
    chassis.moveToPoint(-19,8,1550,{.forwards=false,.maxSpeed=115}); //reverses from 2 stack
    pros::delay(80);
    Matchload.set_value(0);
    chassis.turnToHeading(88,500);
    chassis.moveToPoint(-36,10,600,{.forwards=false});
    chassis.turnToHeading(178,600);
    chassis.moveToPoint(-33,19.7,600,{.forwards=false,.minSpeed=40}); //Backs into long goal
    intake.move(-127);
    Matchload.set_value(1);
    intake.move(0);
    chassis.waitUntilDone();
    intake.move(127);
    pros::delay(1500);
    intake.move(0);
    chassis.moveToPoint(-31,-16.5,1480,{.maxSpeed=55}); //Goes into matchload
    intakeFront.move(127);
    chassis.moveToPoint(-28.2,-4.2,500,{.forwards=false,.minSpeed=40,.earlyExitRange=3}); //Backs out of matchload 
    chassis.waitUntil(5);
    chassis.swingToHeading(221,lemlib::DriveSide::LEFT,800,{.direction=lemlib::AngularDirection::CW_CLOCKWISE,.maxSpeed=127,.minSpeed=50,.earlyExitRange=3}); //Swingsto the angle of middle goal
    chassis.moveToPoint(7.2,38.5,1150,{.forwards=false,.maxSpeed=86}); //Backs into middle goal
    pros::delay(100);
    intake.move(-110);
    pros::delay(100);
    intake.move(0);
    chassis.waitUntil(33.6);
    middleGoal.set_value(1);
    pros::delay(45);
    intakeFront.move(100);
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.moveToPoint(-17.6,11,1100); //Drives towards matchload and long goal
    middleGoal.set_value(0);
    chassis.turnToHeading(178,350); //Turns to the back of long goal
    Descore.set_value(0);
    chassis.moveToPoint(-21.4,40,1200,{.forwards=false,.maxSpeed=90}); //Pushes bloncks into long goal
    chassis.swingToHeading(191,lemlib::DriveSide::RIGHT,1200,{.minSpeed=50}); //Swings to get stuck into long goal
    MatchloadMech++;
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