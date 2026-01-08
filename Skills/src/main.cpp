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
	intake.move(127);
	chassis.setPose(0,0,0);
	chassis.moveToPoint(0,32.3,1000,{.maxSpeed=127}); //In front of first matchload
	chassis.turnToHeading(86,1000);//turn to Marchload 1
	Matchload.set_value(1);
	chassis.moveToPoint(16,39.75,2600,{.maxSpeed=60}); //Goes into Matchload
	pros::delay(500);
	chassis.moveToPoint(0,40.5,850,{.forwards=false,.maxSpeed=127});//Pulls out of Matchload
	chassis.waitUntilDone();
	Matchload.set_value(0);
	chassis.turnToHeading(-54,1000); //Turn To ally
	chassis.moveToPoint(-12,49,1000,{.maxSpeed=127}); //Drive into the ally
	chassis.turnToHeading(-90,1000,{}); //Turn down the ally
	chassis.moveToPoint(-77,53,2000,{.maxSpeed=100}); //Drive down ally
	chassis.turnToHeading(-135,1000); //Turn to second matchload
	chassis.moveToPoint(-93,44.25,900,{.maxSpeed=127}); //Drive infront of matchload
	chassis.turnToHeading(-88,1000); //Turns to Long Goal
	chassis.moveToPoint(-60,40.5,3250,{.forwards=false,.maxSpeed=50}); //Drive into Long Goal
	chassis.waitUntil(18);
	intake.move(-127);
	pros::delay(100);
	intake.move(127);
	Hood.set_value(1);
	chassis.waitUntilDone();
	Matchload.set_value(1);
	chassis.moveToPoint(-125,40.5,3200,{.maxSpeed=50}); //Moves into Matchload 2
	chassis.waitUntil(10);
	Hood.set_value(0);
	pros::delay(600);
	chassis.moveToPoint(-67,40,3200,{.forwards=false,.maxSpeed=50}); //Scores in long 1 for second time
	chassis.waitUntil(26);
	intake.move(-127);
	pros::delay(100);
	intake.move(127);
	Hood.set_value(1);
	Matchload.set_value(0);
	chassis.moveToPoint(-84,40.5,1000,{.maxSpeed=77}); //Moves out of Long Goal 1
	chassis.waitUntil(5);
	Hood.set_value(0);
	chassis.turnToHeading(-178,1000); //Turns to other side of field
	chassis.moveToPoint(-92,-53,2900,{.maxSpeed=100}); //Drives to in front of matchload 3
	chassis.turnToHeading(-90,1000); //turns to matchload 3
	Matchload.set_value(1);
	chassis.moveToPoint(-115.5,-59.5,3450,{.maxSpeed=45}); //goes into matchload 3
	pros::delay(100);
	chassis.moveToPoint(-99,-60.5,900,{.forwards=false,.maxSpeed=77}); //pulls out of matchload 3
	chassis.turnToHeading(-223,1000);//turns to ally
	Matchload.set_value(0);
	chassis.moveToPoint(-87,-68.87,900,{.maxSpeed=77}); //goes into ally
	chassis.turnToHeading(-270,1000); //turns into ally
	chassis.moveToPoint(-20,-76,2700,{.maxSpeed=77}); //drives through ally
	chassis.turnToHeading(-300,900); //turns to matchload 4
	chassis.moveToPoint(0,-66,1000,{.maxSpeed=77}); //Drive infront of matchload and long goal 2
	chassis.turnToHeading(92,900); //turns to long goal 2
	chassis.moveToPoint(-30.5,-62,3000,{.forwards=false,.maxSpeed=50}); //Reverses into Long goal 2
	chassis.waitUntil(24);
	intake.move(-127);
	pros::delay(100);
	intake.move(127);
	Hood.set_value(1);
	Matchload.set_value(1);
	chassis.moveToPoint(12.5,-62.5,3100,{.maxSpeed=50}); //Moves into Matchload 4
	chassis.waitUntil(10);
	Hood.set_value(0);
	chassis.moveToPoint(-30.5,-60.4,3500,{.forwards=false,.maxSpeed=50}); //backs out of matchload 4 into long goal 2
	chassis.waitUntil(24);
	intake.move(-127);
	pros::delay(100);
	intake.move(127);
	Hood.set_value(1);
	Matchload.set_value(0);
	chassis.moveToPoint(-8,-63,900,{.maxSpeed=77}); //Moves out of Long Goal 2
	chassis.turnToHeading(-324,900); //Turns to red zone
	chassis.moveToPoint(9.3,-50.5,900,{.maxSpeed=77}); //Gets ready to turn to red
	chassis.swingToHeading(1,lemlib::DriveSide::LEFT,1000,{}); //Swing to red zone
	chassis.moveToPoint(16,-40, 1000,{.maxSpeed=77}); //Drives infront of red zone
	chassis.waitUntilDone();
	Matchload.set_value(1);
	chassis.moveToPoint(16,-32, 500,{.maxSpeed=77});
	chassis.moveToPoint(16,-19,3000,{.minSpeed=127}); //Drives into redzone and clears
	chassis.waitUntil(7);
	intake.move(-127);
	pros::delay(200);
	Matchload.set_value(0);
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
		int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // move the robot
		chassis.arcade(rightY, rightX);                 // Sets right motor voltage  
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
