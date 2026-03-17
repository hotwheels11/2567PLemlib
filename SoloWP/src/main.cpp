#include "main.h"
#include "Autos.hpp"
#include "ExternalSystems.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "snapshotPose.hpp"

void initialize() {
    //pros::lcd::initialize();
	chassis.calibrate();// make sure chassis is ready
    pros::Task coordDisplayTask(Coords);
    chassis.setPose(0, 0, 0);
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
    Descore.set_value(1);
    intakeFront.move(127);
    chassis.moveToPoint(0, 7.5, 500,{.maxSpeed=127}); //Pushes Robot
    intakeFront.move(127);
    chassis.moveToPoint(0,-48.2,1780,{.forwards=false,.maxSpeed=100}); //Reverses to Matchload
    intakeFront.move(127);
    chassis.turnToHeading(-90,510,{.minSpeed=20}); //Turns to face Matchload
    Matchload.set_value(1);
    chassis.moveToPoint(-12.6,-49.7,1210,{.maxSpeed=60}); //Matchload 1
    intakeFront.move(127);
    chassis.moveToPoint(25,-50.56,1100,{.forwards=false,.maxSpeed=80}); // Scores in Goal 1
    intake.move(0);
    chassis.waitUntil(25.5);
    intake.move(127);
    chassis.waitUntilDone();
    pros::delay(730);
    intakeTop.move(0);
    Matchload.set_value(0);
    chassis.moveToPoint(10,-48.4,500,{.maxSpeed=110,.minSpeed=50,.earlyExitRange=2}); //Drives out of goal
    intake.move(-127);
    pros::delay(200);
    intakeTop.move(0);
    intakeFront.move(127);
    chassis.turnToHeading(37.5,500); //turns to 3 stack
    chassis.moveToPoint(25.3,-18.8,900,{.minSpeed=50,.earlyExitRange=2}); //Goes to 3 stack
    chassis.turnToHeading(0.67,340,{}); //Turns to 3 stack #2
    chassis.waitUntilDone();
    chassis.moveToPoint(25,24.6,1200,{.maxSpeed=110,.minSpeed=50,.earlyExitRange=4}); //Drives to 3 stack #2
    chassis.waitUntil(29);
    Matchload.set_value(1);
    chassis.turnToHeading(-49,400); //Turns to middle goal
    chassis.moveToPoint(36.05,11,750,{false,110}); //Reverses into middle goal
    pros::delay(50);
    intake.move(-100);
    pros::delay(120);
    intake.move(0);
    middleGoal.set_value(1);
    chassis.waitUntilDone();
    intakeTop.move(40);
    intakeFront.move(87); //Scores in middle
    pros::delay(1150);
    intake.move(0);
    middleGoal.set_value(0);
    chassis.moveToPoint(-3.4,49.5,1220); //Drives infront matchload
    Matchload.set_value(0);
    intake.move(0);
    chassis.turnToHeading(-90,480); //Turns infront of matchlaod #2
    Matchload.set_value(1);
    chassis.moveToPoint(-18.6,48,1100,{.maxSpeed=80}); //Goes into matchload #2
    intakeFront.move(127);
    chassis.moveToPoint(25,46.5,1050,{false}); //Backs into long goal and scores
    chassis.waitUntil(24.5);
    intake.move(127);
    descore++;
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
            intakeTop.move(40);
            intakeFront.move(62);
        }
        else {
            intake.move(0);
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            chassis.setPose(0,0,chassis.getPose().theta);
            do_snapshot(Quadrant::ANY);
        }

        pros::delay(20);
    }
}