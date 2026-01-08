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

class AutoTuner {
private:
    struct TestResult {
        double kP, kI, kD;
        double settlingTime;
        double overshoot;
        double steadyStateError;
        double score; // Lower is better
    };
    
    std::vector<TestResult> results;
    pros::Controller controller;
    
public:
    AutoTuner() : controller(pros::E_CONTROLLER_MASTER) {}
    
    // Test a set of PID gains
    TestResult testGains(double kP, double kI, double kD, double targetDistance) {
        TestResult result = {kP, kI, kD, 0, 0, 0, 0};
        
        // Set the gains
        lemlib::Chassis* chassis = &::chassis; // Your global chassis object
        
        double startTime = pros::millis();
        double maxPos = 0;
        double settledTime = 0;
        bool hasSettled = false;
        
        // Move the robot
        chassis->moveToPose(targetDistance, 0, 0, 2000);
        
        // Monitor the movement
        while (chassis->isInMotion() || pros::millis() - startTime < 3000) {
            double currentPos = chassis->getPose().x;
            double error = std::abs(targetDistance - currentPos);
            
            // Track maximum position (for overshoot)
            if (currentPos > maxPos) maxPos = currentPos;
            
            // Check if settled (error < 1 inch for 200ms)
            if (error < 1.0) {
                if (!hasSettled) {
                    settledTime = pros::millis();
                    hasSettled = true;
                } else if (pros::millis() - settledTime > 200) {
                    result.settlingTime = (pros::millis() - startTime) / 1000.0;
                    result.steadyStateError = error;
                    break;
                }
            } else {
                hasSettled = false;
            }
            
            pros::delay(10);
        }
        
        // Calculate overshoot
        result.overshoot = std::max(0.0, maxPos - targetDistance);
        
        // Calculate score (weighted combination)
        result.score = result.settlingTime * 1.0 + 
                       result.overshoot * 2.0 + 
                       result.steadyStateError * 3.0;
        
        return result;
    }
    
    // Ziegler-Nichols Method
    TestResult zieglerNichols() {
        controller.print(0, 0, "ZN: Finding Ku...");
        
        double kP = 0.1;
        double oscillationDetected = false;
        double ku = 0;
        
        // Increase P until oscillation
        while (kP < 10.0 && !oscillationDetected) {
            controller.print(1, 0, "Testing kP: %.2f", kP);
            
            TestResult test = testGains(kP, 0, 0, 24);
            
            // Check for oscillation (overshoot > 20% of target)
            if (test.overshoot > 4.8) {
                ku = kP;
                oscillationDetected = true;
            }
            
            kP += 0.2;
            pros::delay(500);
        }
        
        if (!oscillationDetected) {
            controller.print(2, 0, "ZN Failed!");
            return {0, 0, 0, 0, 0, 0, 999};
        }
        
        // Calculate PID gains using Ziegler-Nichols
        TestResult result;
        result.kP = 0.6 * ku;
        result.kI = 1.2 * ku / 2.0; // Tu approximated
        result.kD = 0.075 * ku * 2.0;
        
        controller.print(2, 0, "ZN: P=%.2f I=%.3f D=%.2f", 
                        result.kP, result.kI, result.kD);
        
        return result;
    }
    
    // Grid Search Method (more thorough)
    TestResult gridSearch() {
        controller.print(0, 0, "Grid Search...");
        
        std::vector<double> pValues = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0};
        std::vector<double> iValues = {0.0, 0.001, 0.005, 0.01, 0.05};
        std::vector<double> dValues = {0.0, 0.1, 0.2, 0.5, 1.0};
        
        TestResult best = {0, 0, 0, 999, 999, 999, 999};
        int testCount = 0;
        int totalTests = pValues.size() * 3; // Test subset
        
        // Coarse search on P
        for (double p : pValues) {
            controller.print(1, 0, "Test %d/%d", ++testCount, totalTests);
            TestResult test = testGains(p, 0, 0, 24);
            
            if (test.score < best.score) {
                best = test;
            }
            pros::delay(500);
        }
        
        // Fine-tune around best P
        double bestP = best.kP;
        
        // Add D
        for (double d : dValues) {
            controller.print(1, 0, "Test %d/%d", ++testCount, totalTests);
            TestResult test = testGains(bestP, 0, d, 24);
            
            if (test.score < best.score) {
                best = test;
            }
            pros::delay(500);
        }
        
        // Add I
        for (double i : iValues) {
            controller.print(1, 0, "Test %d/%d", ++testCount, totalTests);
            TestResult test = testGains(bestP, i, best.kD, 24);
            
            if (test.score < best.score) {
                best = test;
            }
            pros::delay(500);
        }
        
        controller.print(2, 0, "Best: P=%.2f I=%.3f D=%.2f", 
                        best.kP, best.kI, best.kD);
        
        return best;
    }
    
    // Manual tuning helper with live feedback
    void manualTuneHelper() {
        double kP = 1.0, kI = 0.0, kD = 0.0;
        double step = 0.1;
        int mode = 0; // 0=P, 1=I, 2=D
        
        while (true) {
            controller.clear();
            
            // Display current gains
            const char* modeStr[] = {"P", "I", "D"};
            controller.print(0, 0, "Tuning %s (%.3f)", modeStr[mode], 
                           mode == 0 ? kP : mode == 1 ? kI : kD);
            controller.print(1, 0, "P:%.2f I:%.3f D:%.2f", kP, kI, kD);
            controller.print(2, 0, "A=Test X=Mode ^v=Adj");
            
            // Wait for input
            while (true) {
                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
                    // Test current gains
                    controller.print(1, 0, "Testing...");
                    TestResult test = testGains(kP, kI, kD, 24);
                    controller.print(2, 0, "T:%.1fs O:%.1f E:%.1f", 
                                   test.settlingTime, test.overshoot, test.steadyStateError);
                    pros::delay(2000);
                    break;
                }
                
                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                    // Switch mode
                    mode = (mode + 1) % 3;
                    break;
                }
                
                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
                    if (mode == 0) kP += step;
                    else if (mode == 1) kI += step * 0.01;
                    else kD += step;
                    break;
                }
                
                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                    if (mode == 0) kP = std::max(0.0, kP - step);
                    else if (mode == 1) kI = std::max(0.0, kI - step * 0.01);
                    else kD = std::max(0.0, kD - step);
                    break;
                }
                
                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                    return; // Exit
                }
                
                pros::delay(20);
            }
        }
    }
};

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
	Descore.set_value(1);
	intake.move(127);
	chassis.setPose(0,0,0);
	chassis.moveToPoint(0,16.5,500,{});
	chassis.swingToHeading(-50, lemlib::DriveSide::LEFT,500,{.earlyExitRange=3});//Swing to first 3 blocks
	chassis.waitUntil(7);
	Matchload.set_value(1); //Deploys grabbing blocks
	chassis.waitUntilDone();
	chassis.moveToPoint(-3,25,500);
	chassis.turnToHeading(-128,500); //Turns to Matchload
	chassis.moveToPoint(-26.7,11,1000,{.maxSpeed=77}); //Drives infront of Matchload
	chassis.turnToHeading(-178,500); //Turns to Matchloader
	chassis.moveToPoint(-31.7,-10,1250,{.maxSpeed=50}); //Goes into Matchloader slowly
	chassis.moveToPoint(-31.3,30,3500,{.forwards=false,.maxSpeed=40}); //Drives into long goal
	chassis.waitUntil(27);
	Hood.set_value(1);
	Matchload.set_value(0);
	chassis.waitUntilDone();
	chassis.moveToPoint(-31.3,20,500);
	chassis.waitUntilDone();
	Hood.set_value(0);
	intake.move(0);
	chassis.turnToHeading(265,700);
	chassis.moveToPoint(-18,11,1200,{.forwards=false,.maxSpeed=77});
	chassis.turnToHeading(180,700);
	Descore.set_value(0);
	chassis.moveToPoint(-18,43,2000,{.forwards=false,.maxSpeed=50,});

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
