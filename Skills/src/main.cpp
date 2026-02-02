#include "main.h"
#include "Autos.hpp" // IWYU pragma: keep
#include "ExternalSystems.hpp" // IWYU pragma: keep
#include "distanceReset/snapshot_bindings.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/api.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "distanceReset/snapshot_pose.hpp"   // pulls in collision_map + the template impl
#include <vector>
#include <cmath>

// -------------------------------------------------------
// Snapshot reset lives HERE in main.cpp so it uses the
// exact same sensor objects that ExternalSystems.hpp
// initialized.  No extern, no separate .cpp, no linker
// surprises on PROS.
// -------------------------------------------------------

// Thin wrapper so snapshot_setpose<> can call setPose on
// the chassis.  Converts JAR coords -> LemLib coords.
struct LemLibOdomWrapper {
    void set_position(float x_jar, float y_jar, float heading_jar,
                      float /*fwd*/, float /*side*/) {
        float x_lemlib     = x_jar  - 72.0f;
        float y_lemlib     = y_jar  - 72.0f;
        float h_lemlib     = 90.0f  - heading_jar;
        while (h_lemlib >  180.0f) h_lemlib -= 360.0f;
        while (h_lemlib < -180.0f) h_lemlib += 360.0f;
        chassis.setPose(x_lemlib, y_lemlib, h_lemlib);
    }
};

// Build the sensor config list using the ACTUAL sensor objects from
// ExternalSystems.hpp.  Call once; cached in a static vector.
static std::vector<snapshot_pose::DistanceSensorConfig>& get_sensors() {
    static std::vector<snapshot_pose::DistanceSensorConfig> sensors;
    static bool built = false;
    if (built) return sensors;
    built = true;

    sensors.reserve(3);

    // LEFT sensor (port 17) — facing left
    {
        snapshot_pose::DistanceSensorConfig s{};
        s.dev            = &left_sensor;        // <-- direct pointer, same object
        s.x_right_in     = -5.0f;
        s.y_fwd_in       =  0.0f;
        s.rel_deg        = -90.0f;
        s.field_mask_override = snapshot_pose::MAP_PERIMETER;
        s.use_confidence_gate = true;
        s.min_confidence      = 35;
        sensors.push_back(s);
    }

    // BACK LEFT sensor (port 9) — facing back
    {
        snapshot_pose::DistanceSensorConfig s{};
        s.dev            = &back_left_sensor;
        s.x_right_in     = -3.0f;
        s.y_fwd_in       = -6.0f;
        s.rel_deg        = 180.0f;
        s.field_mask_override = snapshot_pose::MAP_PERIMETER;
        s.use_confidence_gate = true;
        s.min_confidence      = 35;
        sensors.push_back(s);
    }

    // BACK RIGHT sensor (port 21) — facing back
    {
        snapshot_pose::DistanceSensorConfig s{};
        s.dev            = &back_right_sensor;
        s.x_right_in     =  3.0f;
        s.y_fwd_in       = -6.0f;
        s.rel_deg        = 180.0f;
        s.field_mask_override = snapshot_pose::MAP_PERIMETER;
        s.use_confidence_gate = true;
        s.min_confidence      = 35;
        sensors.push_back(s);
    }

    return sensors;
}

// One-shot config (also cached).
static snapshot_pose::SnapshotConfig& get_cfg() {
    static snapshot_pose::SnapshotConfig cfg;
    static bool built = false;
    if (built) return cfg;
    built = true;

    cfg.field_mask            = snapshot_pose::MAP_PERIMETER;
    cfg.candidates_per_sensor = 1;
    cfg.samples               = 3;       // median of 3
    cfg.sample_delay_ms       = 10;      // 3 sensors × 3 samples × 10 ms = 90 ms total
    cfg.max_chi2_per_sensor   = 9.0f;

    return cfg;
}

// Convert LemLib heading -> JAR heading (0=north, CW+), 0-360 range.
static float heading_to_jar() {
    float jar = 90.0f - chassis.getPose().theta;
    while (jar <   0.0f) jar += 360.0f;
    while (jar >= 360.0f) jar -= 360.0f;
    return jar;
}

// The single entry point your code calls.  Mirrors what
// snapshot_setpose_quadrant() used to do, but entirely
// local — no extern sensors involved.
static snapshot_pose::SnapshotResult do_snapshot_reset(
    snapshot_pose::Quadrant q = snapshot_pose::Quadrant::AUTO)
{
    // --- guess pose (JAR coords, origin = bottom-left) ---
    float gx, gy;
    switch (q) {
        case snapshot_pose::Quadrant::BOTTOM_LEFT:  gx=36;  gy=36;  break;
        case snapshot_pose::Quadrant::BOTTOM_RIGHT: gx=108; gy=36;  break;
        case snapshot_pose::Quadrant::TOP_LEFT:     gx=36;  gy=108; break;
        case snapshot_pose::Quadrant::TOP_RIGHT:    gx=108; gy=108; break;
        default: // AUTO — convert current odom to JAR
            gx = chassis.getPose().x + 72.0f;
            gy = chassis.getPose().y + 72.0f;
            break;
    }

    LemLibOdomWrapper wrapper;

    return snapshot_pose::snapshot_setpose(
        wrapper,
        get_sensors(),
        get_cfg(),
        heading_to_jar(),
        0.0f,   // fwd tracker — unused by LemLib wrapper
        0.0f,   // side tracker — unused
        gx, gy
    );
}

// -------------------------------------------------------
// Everything below is your original main.cpp unchanged
// -------------------------------------------------------

void initialize() {
    //pros::lcd::initialize();
    chassis.calibrate();// make sure chassis is ready
    pros::Task coordDisplayTask(Coords);  // back to original - don't change this yet
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

    chassis.turnToHeading(90, 700,{.maxSpeed=30,.minSpeed=30,.earlyExitRange=10});
    chassis.turnToHeading(180, 700,{.maxSpeed=30,.minSpeed=30,.earlyExitRange=10});
    chassis.turnToHeading(270, 700,{.maxSpeed=30,.minSpeed=30,.earlyExitRange=10});
    chassis.turnToHeading(360, 700,{.maxSpeed=30});

    while (true) {
        logCurrentXY();

        double currentHeading = chassis.getPose().theta;
        if (fabs(angleError(360, currentHeading)) < HEADING_TOL) {
            break;
        }

        pros::delay(LOG_INTERVAL);
    }

    pros::delay(50);
    logCurrentXY();

    printDesmosOneLine();
}

void autonomous() {
    //chassis.setPose(0, 0, 0);
    autoDistanceReset(chassis);
    /*
    Descore.set_value(1);
    intakeFront.move(127);
    chassis.moveToPoint(0,15.5,500,{.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
    chassis.swingToHeading(-39,lemlib::DriveSide::LEFT,500,{.maxSpeed=127});
    chassis.moveToPoint(-10,21.5,900);
    chassis.waitUntil(6.5);
    intakeFront.move(0);
    chassis.turnToHeading(-133,500);
    intake.move(-35);
    chassis.moveToPoint(9.3,36.8,900,{.forwards=false});
    Matchload.set_value(1);
    intake.move(0);
    chassis.waitUntilDone();
    middleGoal.set_value(1);
    pros::delay(50);
    Matchload.set_value(0);
    intakeFront.move(100);
    pros::delay(1000);
    chassis.moveToPoint(-29.75,-1.6,1300);
    intake.move(-127);
    pros::delay(100);
    intake.move(127);
    chassis.turnToHeading(-180,500);
    Matchload.set_value(1);
    pros::delay(200);
    intake.move(0);
    intakeFront.move(127);
    middleGoal.set_value(0);
    chassis.moveToPoint(-29.25,-15,2300,{true,50,50},true);
    pros::delay(2300);
    pros::delay(200);
    chassis.moveToPoint(-29,-0,800,{.forwards=false});
    chassis.waitUntilDone();
    Matchload.set_value(0);
    chassis.turnToHeading(-50,600);
    chassis.moveToPoint(-45.5,14,1000);
    intakeFront.move(0);
    chassis.turnToHeading(-1,500);
    middleGoalDescore.set_value(1);
    chassis.moveToPoint(-46.5,90,2200,{.maxSpeed=100});
    chassis.turnToHeading(-90,700);
    middleGoalDescore.set_value(0);
    chassis.moveToPoint(-32,88.5,1000,{.forwards=false});
    chassis.turnToHeading(-1,700);
    chassis.moveToPoint(-34,70,1200,{.forwards=false});
    chassis.waitUntil(18);
    intake.move(127);
    pros::delay(400);
    intake.move(-127);
    pros::delay(100);
    intake.move(127);
    pros::delay(1800);
    Matchload.set_value(1);
    chassis.moveToPoint(-35,115,4000,{.maxSpeed=40,.minSpeed=40},true);
    intakeTop.move(0);
    pros::delay(4000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-34,70,1000,{.forwards=false,.maxSpeed=50});
    chassis.waitUntil(2);
    intake.move(-127);
    pros::delay(50);
    intake.move(0);
    chassis.waitUntilDone();
    intake.move(127);
    pros::delay(1000);
    pros::delay(1200);
    Matchload.set_value(0);
    chassis.moveToPoint(-33.4,97.8,800); 
    chassis.turnToHeading(60,500);
    chassis.moveToPoint(-3,120,1000);
    chassis.swingToHeading(87,lemlib::DriveSide::RIGHT,500,{.minSpeed=100,.earlyExitRange=5});
    intakeFront.move(127);
    chassis.waitUntil(5);
    Matchload.set_value(1); 
    chassis.moveToPoint(44,124,5000,{.maxSpeed=127,.minSpeed=100});
    chassis.waitUntil(5);
    Matchload.set_value(0);
    chassis.waitUntil(40);
    Matchload.set_value(1);
    */
}  

bool middleGoalState = false;
bool middleGoalManual = false;
inline int middleGoalDescoreState = 0;

void opcontrol() {
    //chassis.setPose(0,0,180);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    Descore.set_value(1);
    color_sensor.set_led_pwm(100);

    while (true) {
        // ===== DRIVE =====
        int leftY  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        chassis.tank(leftY, rightY);

        // ===== DESCORE TOGGLE =====
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

        // ===== MIDDLE GOAL MANUAL TOGGLE (RIGHT) =====
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
            intakeFront.move(80);
        }
        else {
            intake.move(0);
        }

        pros::delay(20);
    }
}