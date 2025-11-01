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

void initialize() {
    pros::lcd::initialize();
    master.set_text(0, 0, "1");

    chassis.calibrate();          // make sure chassis is ready
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
void autonomous() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0,14,2000,{.maxSpeed = 40});
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
