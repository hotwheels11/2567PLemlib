#include "main.h"
#include "Autos.hpp"
#include "ExternalSystems.hpp" // IWYU pragma: keep
#include "robodash/api.h" // IWYU pragma: keep
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

// Global selector instance
void initialize() {
    // Initialize robodash components first
	pros::lcd::initialize();
	chassis.calibrate(); // calibrate sensors

	pros::lcd::register_btn1_cb(on_center_button);
}

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
void competition_initialize() {}

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
	Descore(1);
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
    pros::delay(1000);
    intakeFront.move(127);
    chassis.moveToPoint(36, 29, 1000,{.forwards=false,.maxSpeed=70}); //backs out of matchload area
    pros::delay(1000);
    Matchload.set_value(0); //Deactivates
    Descore(0);
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
 int descore = 1;
 int intakelift = 1;
 int MatchloadMech = 0;
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // move the robot
        chassis.tank(leftY, rightY);                  // Sets right motor voltage  
		std::stringstream ss;
		ss << std::fixed << std::setprecision(1) // change 1 to however many decimals you want
		<< "(" << chassis.getPose().x << ", "
		<< chassis.getPose().y << ", "
		<< chassis.getPose().theta << ")";
		master.set_text(0, 0, ss.str());
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
		descore++;
		if (descore % 2 == 0){
			DescoreLeft.set_value(true);
			DescoreRight.set_value(true);
		}
		}
		if (descore % 2 == 1){
		DescoreLeft.set_value(false);
		DescoreRight.set_value(false);
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
		intakelift++;
		if (intakelift % 2 == 0){
			IntakeLift.set_value(true);
		} 
		}
		if (intakelift % 2 == 1){
		IntakeLift.set_value(false);
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
		} else {
		intake.move(0);
		intakelate.move(0);
		}
	pros::delay(20);    
	}
}