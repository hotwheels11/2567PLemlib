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
    chassis.setPose(0, 0, 270);
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
    SnapshotResult res = do_snapshot(Quadrant::ANY); //Dih reset 
    pros::delay(75);
    Descore.set_value(1);
    intakeFront.move(127);
    chassis.moveToPoint(-4.5, -63.1, 400,{.maxSpeed=127,}); //Pushes Robot
    chassis.moveToPoint(51,-63,1800,{.forwards=false,.maxSpeed=100}); //Reverses to Matchload
    chassis.turnToHeading(-182,450,{.minSpeed=20,.earlyExitRange=3}); //Turns to face Matchload
    Matchload.set_value(1);
    chassis.moveToPoint(53,-76.5,1150,{.maxSpeed=60}); //Matchload 1
    intakeFront.move(127);
    chassis.moveToPoint(54,-40,1300,{.forwards=false,.maxSpeed=80}); // Scores in Goal 1
    intake.move(0);
    chassis.waitUntil(25);
    intake.move(127);
    chassis.waitUntilDone();
    pros::delay(700);
    intakeTop.move(0);
    Matchload.set_value(0);
    chassis.moveToPoint(53.8,-54,500,{.maxSpeed=110,.minSpeed=50,.earlyExitRange=2}); //Drives out of goal
    intake.move(-127);
    pros::delay(200);
    intakeTop.move(0);
    intakeFront.move(127);
    chassis.turnToHeading(307,600,{.minSpeed=50,.earlyExitRange=4}); //turns to 3 stack
    chassis.moveToPoint(24.6,-38,1000,{.maxSpeed=110,.minSpeed=50,.earlyExitRange=4}); //Goes to 3 stack
    chassis.waitUntil(15.5);
    Matchload.set_value(1);
    chassis.turnToHeading(268,400,{}); //Turns to 3 stack #2
    chassis.waitUntilDone();
    Matchload.set_value(0);
    chassis.moveToPoint(-20,-38,1200,{.maxSpeed=110,.minSpeed=50,.earlyExitRange=4}); //Drives to 3 stack #2
    chassis.waitUntil(28);
    Matchload.set_value(1);
    chassis.turnToHeading(232,500,{.minSpeed=40,.earlyExitRange=3}); //Turns to middle goal
    chassis.moveToPoint(-8.75,-27,800,{false,110}); //Reverses into middle goal
    intake.move(-127);
    pros::delay(200);
    intake.move(0);
    middleGoal.set_value(1);
    chassis.waitUntilDone();
    intakeTop.move(43);
    intakeFront.move(80); //Scores in middle
    pros::delay(1000);
    intake.move(0);
    middleGoal.set_value(0);
    chassis.moveToPoint(-46.25,-64,1200,{.minSpeed=40,.earlyExitRange=4}); //Drives infront matchload
    intake.move(0);
    chassis.turnToHeading(182,500,{.minSpeed=40,.earlyExitRange=3}); //Turns infront of matchlaod #2
    chassis.moveToPoint(-44.9,-83,1100,{.maxSpeed=80}); //Goes into matchload #2
    intakeFront.move(127);
    chassis.moveToPoint(-43.7,-40,1000,{false}); //Backs into long goal and scores
    chassis.waitUntil(23);
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