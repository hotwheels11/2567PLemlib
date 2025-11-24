#include "main.h"
#include "Autos.hpp"
#include "ExternalSystems.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"


void initialize() {
    //pros::lcd::initialize();
	ts::selector::get()->display();
	chassis.calibrate();// make sure chassis is ready
    pros::Task coordDisplayTask(Coords);
}

ts::auton GC("Get Cords", calculatePoseAuto);
ts::auton WPLU("WP Auto Line Up", line_Up_Solo);
ts::auton WP("Solo WP Auto", WP_Auto);
ts::auton LU("Line Up", line_Up_Auto);
ts::auton RS("Right Side", right_Side);
ts::auton LS("Left Side", left_Side);
ts::auton SK("Skills", Skills_Auto);


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
	ts::selector::get()->display();
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
	pros::delay(50);
	Descore.set_value(1);
	chassis.moveToPoint(-1,29,750,{.maxSpeed=90,.earlyExitRange=1});
	intake.move(127);
	chassis.swingToHeading(-50,lemlib::DriveSide::LEFT,500,{.maxSpeed=80});
	chassis.waitUntil(3.1);
	Matchload.set_value(1);
	chassis.waitUntilDone();
	chassis.turnToHeading(-33,350);
	chassis.moveToPoint(-13,53,1350,{.maxSpeed=80});
	chassis.swingToHeading(-81,lemlib::DriveSide::LEFT,300,{.maxSpeed=80});
	Matchload.set_value(0);
	chassis.moveToPoint(-26,61,500,{.maxSpeed=50});
	chassis.waitUntil(5.7);
	Matchload.set_value(1);
	chassis.waitUntilDone();
	chassis.moveToPoint(-6,46,1200,{.forwards=false,.maxSpeed=60});
	chassis.waitUntilDone();
	Matchload.set_value(0);
	chassis.turnToHeading(-143, 700);
	chassis.waitUntilDone();
	chassis.moveToPoint(-29, 23.3, 1100,{.maxSpeed=90});
	chassis.waitUntilDone();
	chassis.turnToHeading(-180, 800);
	chassis.moveToPoint(-31, 48, 2600,{.forwards=false,.maxSpeed=55});
	chassis.waitUntil(11);
	intake.move(-127);
	pros::delay(220);
	intake.move(127);
	Hood.set_value(1);
	chassis.waitUntilDone();
	intake.move(0);
	chassis.waitUntilDone();
	Matchload.set_value(1);
	intake.move(127);
	chassis.moveToPoint(-31,12,500,{.earlyExitRange=3});
	chassis.moveToPoint(-31.5,2,1150,{.maxSpeed=50});
	Hood.set_value(0);
	chassis.waitUntilDone();
	chassis.moveToPoint(-30, 49, 2950, {.forwards=false,.maxSpeed=55});
	chassis.waitUntil(23);
	intake.move(-127);
	pros::delay(250);
	intake.move(127);
	Hood.set_value(1);
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
 int descore = 0;
 int park = 0;
 int MatchloadMech = 0;
 int hoodmech=0;

void opcontrol() {
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	Descore.set_value(1);
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // move the robot
        chassis.tank(leftY, rightY);                  // Sets right motor voltage  
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
		descore++;
		if (descore % 2 == 1){
			Descore.set_value(true);
		}

		}
		if (descore % 2 == 0){
		Descore.set_value(0);
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
		hoodmech++;
		if (hoodmech % 2 == 0){
			Hood.set_value(0);
		}
		}
		if (hoodmech % 2 == 1){
		Hood.set_value(1);
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
		park++;
		if (park % 2 == 1){
			Park.set_value(1);
		} 
		}
		if (park % 2 == 0){
		Park.set_value(0);
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
		MatchloadMech++;
		if (MatchloadMech % 2 == 0){
			Matchload.set_value(true);
		}
		}
		if (MatchloadMech % 2 == 1){
		Matchload.set_value(false);
		}
		
		if (master.get_digital(DIGITAL_R2)) {
		intake.move(-127);
		intakelate.move(-127);
		} else if (master.get_digital(DIGITAL_R1)) {
		intake.move(127);
		intakelate.move(127);
		} else if (master.get_digital(DIGITAL_L2)) {
		intake.move(-127);
		intakelate.move(127);
		} else if (master.get_digital(DIGITAL_L1)) {
		intake.move(127);
		intakelate.move(-127);
		} else if (master.get_digital(DIGITAL_A)){
			intake.move(-70);
		} else {
		intake.move(0);
		intakelate.move(0);
		}
		/*
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			intake.move(-70);
		} else {
			intake.move(0);
		}
			*/
	}
	pros::delay(20);    
}
