#include "main.h"
#include "Autos.hpp"
#include "ExternalSystems.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
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
	master.set_text(0,0,"1");
	chassis.calibrate(); // calibrate sensors
	//rd::Selector *selector = new rd::Selector({{"Auto1", line_Up_Auto,}});
	pros::lcd::register_btn1_cb(on_center_button);
	pros::Task coordDisplayTask(Coords);
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
 ASSET(SoloLineUp_txt);
 ASSET(LineUp_txt);
void autonomous() {
	void line_Up_Solo();
	chassis.setPose(-62.469,-16.707,90);
	chassis.moveToPose(-43,-16.3,90,1000,{.maxSpeed=30});
	chassis.waitUntilDone();
	pros::delay(1000);
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
    //pros::delay(100);
    //chassis.follow(LineUp_txt,2.3,1000);
	/*
    chassis.setPose(0, 0, 0);
	Descore.set_value(1);
    chassis.moveToPoint(0, 13, 700);
    intake.move(127);
    chassis.moveToPoint(-9.0, 28, 700);//get first three balls
    chassis.waitUntil(3);
    Matchload.set_value(1); //Activates to hold balls in place
    chassis.waitUntilDone();
    Matchload.set_value(0); //Deactivates
    pros::delay(400);
	chassis.turnToHeading(-50, 800);
    chassis.moveToPoint(-24, 44.6, 1500);
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
    chassis.moveToPoint(-33.2,-5,1500,{.maxSpeed=60}); //Drives into Matchload Area
    pros::delay(750);
    intakeFront.move(127);
    chassis.moveToPoint(-33, 35, 2000,{.forwards=false,.maxSpeed=65}); //backs out of matchload area
    pros::delay(800);
    Matchload.set_value(0); //Deactivates
    Descore.set_value(0);*/
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
	pros::Task ParkTask(enableButtonTask); // Start the background timer when driver begins
	pros::Task DistanceTask(activePosition);
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
		} else {
		intake.move(0);
		intakelate.move(0);
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			if (buttonEnabled) {
				intake.move(-127);  // Start intake

				// Wait until inPosition becomes true
				while (!inPosition) {
					pros::delay(10);  // small delay to avoid hogging CPU
				}

				intake.move(0);       // Stop intake
				park=1;
				Park.set_value(1);
				pros::delay(100);

				inPosition = false;   // Reset for next use 
			} else {
				master.rumble(".."); // Not ready yet
			}
		}

	pros::delay(20);    
	}
}