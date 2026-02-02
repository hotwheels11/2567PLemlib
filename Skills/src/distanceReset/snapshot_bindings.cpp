// src/distanceReset/snapshot_bindings.cpp
//
// Snapshot Pose "bindings" layer:
// - Keeps all configuration in ONE place.
// - Your auton should only call snapshot_pose::snapshot_setpose_simple().
//
// What this does:
// - Reads 2–3 Distance sensors (median-filtered).
// - Uses raycasts against the collision map segments to infer robot (x,y) on the field.
// - Applies chassis.setPose(x, y, heading) for LemLib.
//
// Reliability defaults:
// - Perimeter-only by default (walls are the most stable landmarks).
// - Per-sensor masks supported: low sensors can ignore interior objects that they can't see.
//
// Coordinate system (JAR to LemLib conversion):
// - Field is 144" x 144".
// - JAR: (0,0) bottom-left, +X right, +Y up, heading 0° = +Y (north), clockwise-positive
// - LemLib: (0,0) center, +X right, +Y up, heading 0° = +X (east), counter-clockwise positive
// - This file handles the conversion automatically

#include "distanceReset/snapshot_bindings.hpp"
#include "lemlib/api.hpp"
#include "pros/distance.hpp"
#include "pros/llemu.hpp"

// Declare only what we need from ExternalSystems.hpp as extern.
// Do NOT #include "ExternalSystems.hpp" here - all those inline globals
// would get duplicated into this compilation unit and can cause hangs on PROS.
extern lemlib::Chassis chassis;
extern pros::Distance left_sensor;
extern pros::Distance back_left_sensor;
extern pros::Distance back_right_sensor;

namespace snapshot_pose {

// -------------------------------
// LemLib Odometry Wrapper
// -------------------------------
// This wrapper converts between JAR and LemLib coordinate systems
struct LemLibOdomWrapper {
    lemlib::Chassis* chassis_ptr;
    
    // JAR expects: set_position(x_jar, y_jar, heading_jar, fwd_tracker, side_tracker)
    // where x_jar, y_jar are in bottom-left origin, heading_jar is 0=north, CW+
    void set_position(float x_jar, float y_jar, float heading_jar, float fwd_tracker, float side_tracker) {
        // Convert JAR coordinates to LemLib coordinates
        // JAR: (0,0) = bottom-left, LemLib: (0,0) = center
        float x_lemlib = x_jar - 72.0f;  // 72 = field_size/2
        float y_lemlib = y_jar - 72.0f;
        
        // Convert JAR heading to LemLib heading
        // JAR: 0° = north (+Y), CW+
        // LemLib: 0° = east (+X), CCW+
        // Conversion: lemlib_heading = 90 - jar_heading
        float heading_lemlib = 90.0f - heading_jar;
        
        // Normalize to -180 to 180
        while (heading_lemlib > 180.0f) heading_lemlib -= 360.0f;
        while (heading_lemlib < -180.0f) heading_lemlib += 360.0f;
        
        // LemLib setPose takes (x, y, theta) only - no tracker arguments
        chassis_ptr->setPose(x_lemlib, y_lemlib, heading_lemlib);
    }
};

// -------------------------------
// EDIT SECTION 1: DISTANCE SENSORS
// -------------------------------
// Using the sensors defined in ExternalSystems.hpp
// These are already declared as:
// - pros::Distance left_sensor(17);
// - pros::Distance back_left_sensor(9);
// - pros::Distance back_right_sensor(21);

// -------------------------------
// EDIT SECTION 2: HEADING AND TRACKER SOURCES
// -------------------------------
static float get_heading_jar_deg() {
    // Get LemLib heading and convert to JAR convention
    lemlib::Pose pose = chassis.getPose();
    float heading_lemlib = pose.theta;
    
    // Convert: jar_heading = 90 - lemlib_heading
    float heading_jar = 90.0f - heading_lemlib;
    
    // Normalize to 0-360
    while (heading_jar < 0.0f) heading_jar += 360.0f;
    while (heading_jar >= 360.0f) heading_jar -= 360.0f;
    
    return heading_jar;
}

static float get_forward_tracker_in() {
    // LemLib tracks internally - we return 0 since setPose doesn't use this
    return 0.0f;
}

static float get_sideways_tracker_in() {
    // LemLib tracks internally - we return 0 since setPose doesn't use this
    return 0.0f;
}

// -------------------------------
// INTERNAL: configuration + sensor list
// -------------------------------
static SnapshotConfig g_cfg;
static std::vector<DistanceSensorConfig> g_sensors;
static bool g_init = false;

static void init_once() {
    if (g_init) return;
    g_init = true;

    // -------- Global snapshot config (safe defaults) --------
    g_cfg.field_mask = MAP_PERIMETER;        // default fallback if per-sensor override is 0
    g_cfg.candidates_per_sensor = 1;         // perimeter-only: 1 candidate is usually enough
    g_cfg.samples = 3;                       // median of 3 readings (was 5 — reduced to cut blocking time)
    g_cfg.sample_delay_ms = 10;              // 10 ms between reads (was 35 ms)
                                             // Total read time: 3 sensors × 3 samples × 10 ms = 90 ms (was 525 ms)
    g_cfg.max_chi2_per_sensor = 9.0f;        // ~3-sigma per sensor

    // If you enable interior objects globally, increase candidates:
    // g_cfg.field_mask = MAP_PERIMETER | MAP_LONG_GOALS | MAP_CENTER_GOALS;
    // g_cfg.candidates_per_sensor = 2;

    // -------- Sensor definitions --------
    //
    // IMPORTANT: Coordinate system for sensor offsets:
    // - x_right_in: inches from robot center, + to robot-right
    // - y_fwd_in: inches from robot center, + to robot-forward
    //
    // rel_deg (relative to robot forward):
    // - 0 = forward
    // - +90 = right
    // - 180 = back
    // - -90 = left
    //
    // You MUST measure these offsets from YOUR robot's center to each sensor!

    g_sensors.clear();
    g_sensors.reserve(3);

    // LEFT sensor (port 17)
    {
        DistanceSensorConfig s{};
        s.dev = &left_sensor;

        // EDIT: Measure these offsets from YOUR robot!
        // Example values - REPLACE WITH YOUR MEASUREMENTS
        s.x_right_in = -5.0f;  // negative = left of center
        s.y_fwd_in   = 0.0f;   // at robot center front-to-back
        s.rel_deg    = -90.0f; // facing left

        // Per-sensor mask: only trust walls for this sensor
        s.field_mask_override = MAP_PERIMETER;

        // Confidence gating (0..63) only meaningful when distance > 200mm
        s.use_confidence_gate = true;
        s.min_confidence = 35;

        g_sensors.push_back(s);
    }

    // BACK LEFT sensor (port 9)
    {
        DistanceSensorConfig s{};
        s.dev = &back_left_sensor;

        // EDIT: Measure these offsets from YOUR robot!
        // Example values - REPLACE WITH YOUR MEASUREMENTS
        s.x_right_in = -3.0f;  // left of center
        s.y_fwd_in   = -6.0f;  // negative = behind robot center
        s.rel_deg    = 180.0f; // facing back

        s.field_mask_override = MAP_PERIMETER;
        s.use_confidence_gate = true;
        s.min_confidence = 35;

        g_sensors.push_back(s);
    }

    // BACK RIGHT sensor (port 21)
    {
        DistanceSensorConfig s{};
        s.dev = &back_right_sensor;

        // EDIT: Measure these offsets from YOUR robot!
        // Example values - REPLACE WITH YOUR MEASUREMENTS
        s.x_right_in = 3.0f;   // right of center
        s.y_fwd_in   = -6.0f;  // negative = behind robot center
        s.rel_deg    = 180.0f; // facing back

        s.field_mask_override = MAP_PERIMETER;
        s.use_confidence_gate = true;
        s.min_confidence = 35;

        g_sensors.push_back(s);
    }

    pros::lcd::print(6, "SnapshotPose init OK");
}

// -------------------------------
// Internal helper to get guess pose based on quadrant
// -------------------------------
static void get_guess_pose_from_quadrant(Quadrant q, float& guess_x_jar, float& guess_y_jar) {
    // Get current pose from LemLib
    lemlib::Pose current_pose = chassis.getPose();
    
    // Default: convert current LemLib pose to JAR coordinates
    guess_x_jar = current_pose.x + 72.0f;
    guess_y_jar = current_pose.y + 72.0f;
    
    // If quadrant is specified, bias the guess toward that quadrant's center
    switch (q) {
        case Quadrant::BOTTOM_LEFT:
            // Center of bottom-left quadrant in JAR coords
            guess_x_jar = 36.0f;
            guess_y_jar = 36.0f;
            break;
            
        case Quadrant::BOTTOM_RIGHT:
            // Center of bottom-right quadrant in JAR coords
            guess_x_jar = 108.0f;
            guess_y_jar = 36.0f;
            break;
            
        case Quadrant::TOP_LEFT:
            // Center of top-left quadrant in JAR coords
            guess_x_jar = 36.0f;
            guess_y_jar = 108.0f;
            break;
            
        case Quadrant::TOP_RIGHT:
            // Center of top-right quadrant in JAR coords
            guess_x_jar = 108.0f;
            guess_y_jar = 108.0f;
            break;
            
        case Quadrant::AUTO:
            // Use current odometry (already set above)
            break;
    }
}

// -------------------------------
// Public API called by auton
// -------------------------------
SnapshotResult snapshot_setpose_simple() {
    return snapshot_setpose_quadrant(Quadrant::AUTO);
}

SnapshotResult snapshot_setpose_quadrant(Quadrant q) {
    init_once();

    // Get guess pose based on quadrant
    float guess_x_jar, guess_y_jar;
    get_guess_pose_from_quadrant(q, guess_x_jar, guess_y_jar);
    
    // Get heading in JAR convention
    const float heading_deg = get_heading_jar_deg();
    const float fwd_in = get_forward_tracker_in();
    const float side_in = get_sideways_tracker_in();

    // Create LemLib wrapper for odometry updates
    LemLibOdomWrapper odom_wrapper;
    odom_wrapper.chassis_ptr = &chassis;

    // Strongly recommended: call this while the robot is stopped.
    // (Do chassis.waitUntilDone(); pros::delay(150-250); in auton before calling.)

    return snapshot_setpose(
        odom_wrapper,
        g_sensors,
        g_cfg,
        heading_deg,
        fwd_in,
        side_in,
        guess_x_jar,
        guess_y_jar
    );
}

} // namespace snapshot_pose