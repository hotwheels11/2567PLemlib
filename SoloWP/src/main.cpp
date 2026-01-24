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
    Descore.set_value(1);
    intakeFront.move(127);
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 5, 400,{.maxSpeed=127,.minSpeed=127}); //Pushes Robot
    chassis.moveToPoint(0,-47,1900,{.forwards=false,.maxSpeed=100}); //Reverses to Matchload
    chassis.turnToHeading(-88,400,{.minSpeed=20,.earlyExitRange=1}); //Turns to face Matchload
    Matchload.set_value(1);
    chassis.moveToPoint(-24,-54,980,{.maxSpeed=80}); //Matchload 1
    chassis.moveToPoint(21,-55,1880,{.forwards=false,.maxSpeed=80}); // Scores in Goal 1
    chassis.waitUntil(30);
    intakeTop.move(127);
    chassis.waitUntilDone();
    intakeTop.move(0);
    chassis.moveToPoint(0,-53,500,{.maxSpeed=110,.minSpeed=30,.earlyExitRange=2}); //Drives out of goal
    intake.move(-127);
    pros::delay(200);
    intakeTop.move(0);
    intakeFront.move(127);
    chassis.turnToHeading(40,600,{.minSpeed=20,.earlyExitRange=4}); //turns to 3 stack
    Matchload.set_value(0);
    chassis.moveToPoint(32,-21.7,1000,{.maxSpeed=110}); //Goes to 3 stack
    chassis.waitUntil(14);
    Matchload.set_value(1);
    chassis.turnToHeading(0,400,{.minSpeed=20,.earlyExitRange=3}); //Turns to 3 stack #2
    chassis.waitUntilDone();
    Matchload.set_value(0);
    chassis.moveToPoint(28,24.5,1200,{.maxSpeed=110}); //Drives to 3 stack #2
    chassis.waitUntil(28);
    Matchload.set_value(1);
    chassis.turnToHeading(-45,500,{.minSpeed=20,.earlyExitRange=3}); //Turns to middle goal
    chassis.moveToPoint(36.25,9.5,1000,{.forwards=false,.maxSpeed=100,.minSpeed=1,.earlyExitRange=1}); //Backs and scores
    intake.move(-127);
    pros::delay(250);
    intake.move(0);
    middleGoal.set_value(1);
    chassis.waitUntilDone();
    intakeFront.move(110);
    pros::delay(1200);
    intakeFront.move(0);
    middleGoal.set_value(0);
    chassis.moveToPoint(-7,48,1500,{.maxSpeed=110,.minSpeed=20,.earlyExitRange=2}); //Drives to matchload 2
    intake.move(-127);
    pros::delay(100);
    intakeTop.move(0);
    intakeFront.move(127);
    chassis.swingToHeading(-90,lemlib::DriveSide::LEFT,400,{.minSpeed=20,.earlyExitRange=3}); //Swings to matchload 2
    chassis.moveToPoint(-22.75,44,900,{.maxSpeed=80}); //Collects Matchload 2
    chassis.moveToPoint(24.95,45.25,1000,{.forwards=false,.maxSpeed=75}); // Scores in Goal 2
    chassis.waitUntilDone();
    intake.move(127);
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
            intakeFront.move(127);
        }
        else {
            intake.move(0);
        }

        pros::delay(20);
    }
}