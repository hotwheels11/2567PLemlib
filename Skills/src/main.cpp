#include "main.h"
#include "Autos.hpp" // IWYU pragma: keep
#include "ExternalSystems.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "snapshotPose.hpp"


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

 #include "main.h"
#include "lemlib/api.hpp"
#include <vector>
#include "main.h"
#include "lemlib/api.hpp"
#include <vector>
#include <cmath>

// ---------------- SETTINGS ----------------
const int LOG_INTERVAL = 20;      // ms
const double HEADING_TOL = 1.5;   // degrees tolerance

// ---------------- STORAGE -----------------
struct Point {
    double x;
    double y;
};

std::vector<Point> loggedPoints;

// --------------- LOG POSE -----------------
void logCurrentXY() {
    lemlib::Pose pose = chassis.getPose();
    loggedPoints.push_back({pose.x, pose.y});
}

// --------------- DESMOS OUTPUT -------------
void printDesmosOneLine() {
    std::stringstream ss;
    ss << "[";
    for (size_t i = 0; i < loggedPoints.size(); i++) {
        ss << "(" << loggedPoints[i].x << "," << loggedPoints[i].y << ")";
        if (i != loggedPoints.size() - 1) ss << ",";
    }
    ss << "]";
    printf("\n%s\n", ss.str().c_str());
}

// -------- ANGLE NORMALIZATION --------------
double angleError(double target, double current) {
    double error = target - current;
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    return error;
}

// -------- MAIN 360 LOG ROUTINE -------------
void rotate360AndLog() {
    loggedPoints.clear();

    chassis.setPose(0,0,0);  // reset pose
    logCurrentXY();           // starting point

    // Start async 360 turn
    chassis.turnToHeading(90, 700,{.maxSpeed=30,.minSpeed=30,.earlyExitRange=10});
    chassis.turnToHeading(180, 700,{.maxSpeed=30,.minSpeed=30,.earlyExitRange=10});
    chassis.turnToHeading(270, 700,{.maxSpeed=30,.minSpeed=30,.earlyExitRange=10});
    chassis.turnToHeading(360, 700,{.maxSpeed=30});

    while (true) {
        logCurrentXY();

        double currentHeading = chassis.getPose().theta;
        if (fabs(angleError(360, currentHeading)) < HEADING_TOL) {
            break; // finished turning
        }

        pros::delay(LOG_INTERVAL); // 20ms between logs
    }

    pros::delay(50);
    logCurrentXY(); // final pose

    printDesmosOneLine();
}




void test_snapshot() {
    // Make sure robot is completely stopped
    chassis.cancelMotion();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    pros::delay(300);
    
    // Get current pose
    lemlib::Pose cur = chassis.getPose();
    
    // Call snapshot - it will now print detailed debug info
    SnapshotResult res = snapshot_setpose(
        SENSORS,
        DEFAULT_FIELD_MAP,
        DEFAULT_FIELD_MAP_SIZE,
        MASK_PERIMETER,
        cur.x,
        cur.y,
        cur.theta,
        Quadrant::ANY,   // Use ANY to see if it works anywhere first
        read_sensor_cb,
        delay_cb
    );
    
    if (res.ok) {
        chassis.setPose(lemlib::Pose(res.x_in, res.y_in, cur.theta));
    }
}




void autonomous() {
    chassis.setPose(0, 0, 0);
    Descore.set_value(1);
    intakeFront.move(127);
    chassis.moveToPoint(0,15.25,500,{.maxSpeed=127,.minSpeed=20,.earlyExitRange=2}); //Drives towards 4 stack
    chassis.swingToHeading(-40,lemlib::DriveSide::LEFT,460,{.maxSpeed=127}); //Swings into 4 stack
    chassis.moveToPoint(-10,21.5,820); //Drives into 4 stack
    chassis.waitUntil(10);
    intakeFront.move(0);
    chassis.turnToHeading(-133,500); //Turns to Middle Goal
    intake.move(-35);
    chassis.moveToPoint(9.3,36.8,900,{.forwards=false}); //Backs into middle goal
    Matchload.set_value(1);
    intake.move(0);
    chassis.waitUntilDone();
    middleGoal.set_value(1); //Starts the scoring to middle goal
    pros::delay(50);
    Matchload.set_value(0);
    intakeFront.move(100);
    pros::delay(1000);
    chassis.moveToPoint(-31,-1.6,1300); //Drives to matchload #1
    intake.move(-127);
    pros::delay(100);
    intake.move(127);
    chassis.turnToHeading(-178,500); //Turns to matchload #1
    Matchload.set_value(1);
    pros::delay(200);
    intake.move(0);
    intakeFront.move(127);
    chassis.moveToPoint(-31,-15.3,2200,{true,50}); //Goes into matchload #1
    chassis.waitUntil(5);
    middleGoal.set_value(0);
    pros::delay(2000);
    chassis.moveToPoint(-31,-0,800,{.forwards=false}); //Backs out from Matchload #1
    chassis.waitUntilDone();
    Matchload.set_value(0);
    chassis.turnToHeading(-50,600); //Turns to Ally
    chassis.moveToPoint(-46.3,14,1000); //Drives before ally
    intakeFront.move(0);
    chassis.turnToHeading(-1,350); //Turns into ally
    middleGoalDescore.set_value(1);
    chassis.moveToPoint(-47.3,90,2200,{.maxSpeed=90}); //Drives down ally
    chassis.turnToHeading(-90,700); //turns to long goal
    middleGoalDescore.set_value(0);
    chassis.moveToPoint(-34,88.5,1000,{.forwards=false}); //Back up perpendicular to long goal
    chassis.turnToHeading(-1,700); //Backs into long goal
    chassis.moveToPoint(-35,70,1000,{.forwards=false,.maxSpeed=50}); //Backs into long goal
    chassis.waitUntil(18);
    intake.move(127);
    pros::delay(2000);
    intakeTop.move(0);
    Matchload.set_value(1);
    chassis.moveToPoint(-36.75,115,2200,{.maxSpeed=40}); //Drives into matchload #2
    chassis.waitUntil(23);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    pros::delay(2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-35,64.67,1200,{.forwards=false,.maxSpeed=50}); //backs to long goal #1 Second time
    chassis.waitUntil(2);
    intake.move(-127);
    pros::delay(50);
    intake.move(0);
    chassis.waitUntilDone();
    intake.move(127);
    pros::delay(1500);
    Matchload.set_value(0);
    intakeTop.move(0);
    chassis.moveToPoint(-33.5,107,800); // Drive out of long goal
    chassis.turnToHeading(70,500); //TUrns to in the direction of blue park
    chassis.moveToPoint(-4.45,120.85,1100); //Point before swing for blue park
    chassis.swingToHeading(90,lemlib::DriveSide::RIGHT,500,{.minSpeed=80}); //Swings to blue park
    chassis.waitUntilDone();
    Matchload.set_value(1);
    pros::delay(250);
    chassis.moveToPoint(40,126,4000,{.maxSpeed=127,.minSpeed=127}); //Driving through park
    chassis.waitUntil(1.5);
    Matchload.set_value(0);
    chassis.waitUntil(35);
    Matchload.set_value(1);
    chassis.swingToHeading(180,lemlib::DriveSide::RIGHT,1000); //sets up for dih reset
    chassis.waitUntil(30);
    Matchload.set_value(0);
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.setPose(0,0,chassis.getPose().theta);
    pros::delay(20);
    SnapshotResult res = do_snapshot(Quadrant::ANY); //Dih reset 
    pros::delay(400);
    if (res.ok) {
        intake.move(0);
        intakeFront.move(0); //Verifies I can get control of the intake in this while loop
        intakeTop.move(0);
        intakeFront.move(127);
        chassis.turnToHeading(230,500); //Turns in prep to collect 4 balls
        chassis.moveToPoint(13,33,1000); //Moves infront of 4 stack
        chassis.turnToHeading(137,500); //Turns to 4 stack
        chassis.moveToPoint(24,23.7,1000); //collects 4 stack
        chassis.waitUntil(4);
        Matchload.set_value(1);
        chassis.waitUntil(14);
        intake.move(0);
        chassis.waitUntilDone();
        chassis.turnToHeading(41,500); //Turns to middle goal
        chassis.moveToPoint(7.5,7.2,850,{.forwards=false,.maxSpeed=80}); //Backs into middle goal
        //Matchload.set_value(1);
        intake.move(-55);
        chassis.waitUntil(6);
        intake.move(0);
        chassis.waitUntilDone();
        middleGoal.set_value(1); //Starts the scoring to middle goal
        pros::delay(50);
        intakeTop.move(30);
        intakeFront.move(70);
        pros::delay(2100);
        middleGoal.set_value(0);
        intake.move(0);
        chassis.moveToPoint(46.25,50,1100); //Drives infront of Matchload #3
        Matchload.set_value(0);
        chassis.waitUntil(5);
        intake.move(127);
        chassis.turnToHeading(-3,400); //Turns to matchload #3
        chassis.waitUntilDone();
        Matchload.set_value(1);
        pros::delay(200);
        chassis.moveToPoint(45.75,62.5,2200,{true,50}); //Goes into matchload #3
        intakeTop.move(0);
        pros::delay(250);
        intakeFront.move(127);
        pros::delay(1800);
        chassis.moveToPoint(43.5,45.7,750,{.forwards=false}); //Backs out from Matchload #3
        chassis.waitUntil(5);
        Matchload.set_value(0);
        intakeFront.move(0);
        chassis.turnToHeading(135,500); //Turns to Ally
        chassis.moveToPoint(62,37.5,1000); //Drives before ally
        chassis.turnToHeading(176,500); //Turns into ally
        chassis.moveToPoint(65.25,-41.2,2300,{.maxSpeed=100}); //Drives down ally
        chassis.turnToHeading(85,500);
        chassis.moveToPoint(50.5,-40.4,1000,{.forwards=false}); //Back up perpendicular to long goal
        chassis.turnToHeading(174,500); //Turns to long goal
        chassis.moveToPoint(53,-25,1000,{.forwards=false,.maxSpeed=50}); //Backs into long goal
        chassis.waitUntil(18);
        intake.move(127);
        pros::delay(2000);
        chassis.moveToPoint(55.25,-63,2500,{.maxSpeed=40}); //Drives into matchload #4
        intakeTop.move(0);
        Matchload.set_value(1);
        chassis.waitUntil(23);
        pros::delay(2200);
        chassis.moveToPoint(52.75,-26,1400,{.forwards=false,.maxSpeed=50}); //backs to long goal #2 Second time
        chassis.waitUntil(2);
        intake.move(-127);
        pros::delay(50);
        intake.move(0);
        chassis.waitUntilDone();
        intake.move(127);
        pros::delay(1500);
        intake.move(0);
        Matchload.set_value(0);
        chassis.moveToPoint(54.4,-43,800); // Drive out of long goal
        chassis.turnToHeading(226,500); //TUrns to in the direction of red park
        chassis.moveToPoint(25.5,-69,1000); //Point before swing for red park
        chassis.swingToHeading(266,lemlib::DriveSide::RIGHT,500);
        chassis.waitUntilDone();
        Matchload.set_value(1);
        pros::delay(250);
        chassis.moveToPoint(0,-70,3000,{.maxSpeed=127,.minSpeed=127}); //Driving through park
        intake.move(0);
        chassis.waitUntil(1.5);
        Matchload.set_value(0);
    } else if (!res.ok) {
        master.rumble("___");
    }

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