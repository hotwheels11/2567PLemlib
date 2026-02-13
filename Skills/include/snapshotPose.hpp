// ---------------------------------------------------------------------------
// snapshot_pose.h  –  LemLib edition
// ---------------------------------------------------------------------------
// One-shot snapshot pose correction via distance-sensor raycasting.
//
// Sensor layout (your robot):
//   Sensor 0 – back-left   (phi = 180°)
//   Sensor 1 – back-right  (phi = 180°)
//   Sensor 2 – left        (phi = -90°)
//
// Coordinate system  (LemLib default):
//   Origin  = field centre
//   +X      = right
//   +Y      = up  (toward the "top" wall)
//   θ       = degrees, 0° = +Y, positive = clockwise
//   Perimeter walls at x = ±72, y = ±72  (field is 144×144 in)
//
// Heading convention is the same as the paper (0° up, CW+).
// All internal distances are inches; raw sensor mm are converted on intake.
// ---------------------------------------------------------------------------

#pragma once

#include <cstdint>
#include <cmath>
#include <array>
#include <algorithm>
#include "ExternalSystems.hpp"

// =========================================================================
// 1.  Geometry primitives
// =========================================================================

struct Vec2 {
    double x, y;
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(double s)      const { return {x * s,   y * s};   }
    double dot(const Vec2& o)     const { return x * o.x + y * o.y;  }
    double norm()                 const { return std::sqrt(x*x + y*y); }
};

struct MapSegment {
    Vec2     a, b;        // endpoints (inches, LemLib frame)
    uint32_t mask;        // category bitmask
};

// =========================================================================
// 2.  Mask constants
// =========================================================================
static constexpr uint32_t MASK_PERIMETER    = 1u << 0;
static constexpr uint32_t MASK_LONG_GOALS   = 1u << 1;
static constexpr uint32_t MASK_CENTER_GOALS = 1u << 2;
static constexpr uint32_t MASK_ALL          = 0xFFFFFFFFu;

// =========================================================================
// 3.  Default field map  –  perimeter walls, LemLib origin (centre)
// =========================================================================
//
//   (-72, 72) ─────────── (72, 72)      ← top wall
//      │                     │
//      │        (0,0)        │
//      │                     │
//   (-72,-72) ────────── (72,-72)       ← bottom wall
//
static constexpr MapSegment DEFAULT_FIELD_MAP[] = {
    { {-72, -72}, { 72, -72}, MASK_PERIMETER },   // bottom
    { {-72,  72}, { 72,  72}, MASK_PERIMETER },   // top
    { {-72, -72}, {-72,  72}, MASK_PERIMETER },   // left
    { { 72, -72}, { 72,  72}, MASK_PERIMETER },   // right
};
static constexpr int DEFAULT_FIELD_MAP_SIZE = 4;

// =========================================================================
// 4.  Per-sensor configuration
// =========================================================================
struct SensorConfig {
    double   offset_right;     // inches, + = right of chassis centre
    double   offset_fwd;       // inches, + = forward of chassis centre
    double   phi_deg;          // look direction: 0 fwd, 90 right, 180 back, -90 left
    int      port;             // V5 smart port (1-based)
    uint32_t mask_override;    // 0 = inherit global mask
};

// =========================================================================
// 5.  Quadrant enum  (LemLib centre-origin bounds)
// =========================================================================
enum class Quadrant : uint8_t {
    ANY = 0,
    BL  = 1,   // x ∈ [-72,  0]   y ∈ [-72,  0]
    BR  = 2,   // x ∈ [ 0,  72]   y ∈ [-72,  0]
    TL  = 3,   // x ∈ [-72,  0]   y ∈ [  0,  72]
    TR  = 4,   // x ∈ [ 0,  72]   y ∈ [  0,  72]
};

// =========================================================================
// 6.  Tuning defaults  (paper §4.3 / Table 1)
// =========================================================================
static constexpr int    SNAP_SAMPLES         = 5;
static constexpr int    SNAP_DELAY_MS        = 35;
static constexpr double SNAP_RANGE_MIN_IN    = 0.787;   // 20 mm
static constexpr double SNAP_RANGE_MAX_IN    = 78.74;   // 2000 mm
static constexpr int    SNAP_CONF_MIN        = 35;
static constexpr double SNAP_CONF_DIST_IN    = 7.874;   // 200 mm  – gate only above this
static constexpr int    SNAP_K               = 2;       // candidates per sensor
static constexpr double SNAP_CHI2_MAX        = 9.0;     // per-sensor χ² cap (≈ 3 σ)
static constexpr double SNAP_QUADRANT_MARGIN = 2.0;     // inches
static constexpr int    SNAP_MIN_SENSORS     = 2;

// =========================================================================
// 7.  Number of sensors and result type
// =========================================================================
static constexpr int NUM_SENSORS = 3;   // back-left, back-right, left

struct SnapshotResult {
    bool   ok;
    double x_in;            // solved x (inches, LemLib frame)
    double y_in;            // solved y (inches, LemLib frame)
    double chi2;            // χ² score
    int    sensors_used;
};

// =========================================================================
// 8.  Public API
// =========================================================================

// Raw reading returned by the callback you supply.
struct SensorReading {
    int distance_mm;   // -1 on error
    int confidence;    // 0-63
};

// Function-pointer types so the solver stays framework-agnostic internally.
using ReadSensorFn = SensorReading (*)(int port);
using DelayFn      = void           (*)(int ms);

// Run snapshot.  On success result.ok == true and you call
//   chassis.setPose(lemlib::Pose(result.x_in, result.y_in, current_heading));
//
// COORDINATE SYSTEM MODES:
// 1. Field-centered (default): set field_origin_x = 0, field_origin_y = 0
//    - guess_x, guess_y are already in field coordinates
//    - result.x_in, result.y_in are in field coordinates
//
// 2. Robot-centered: set field_origin_x and field_origin_y to where your
//    robot-centered origin (0,0) maps to in field coordinates
//    - guess_x, guess_y are in robot-centered coordinates
//    - result.x_in, result.y_in are in field coordinates (converted!)
//    Example: if you start in top-right corner at field position (60, 60),
//             set field_origin_x = 60, field_origin_y = 60
SnapshotResult snapshot_setpose(
    const SensorConfig  sensors[NUM_SENSORS],
    const MapSegment*   map,
    int                 map_size,
    uint32_t            global_mask,
    double              guess_x,         // current odom x (inches, robot or field frame)
    double              guess_y,         // current odom y (inches, robot or field frame)
    double              heading_deg,     // current heading (degrees, LemLib convention)
    Quadrant            quadrant,
    ReadSensorFn        read_sensor,
    DelayFn             delay_fn,
    double              field_origin_x = 0.0,  // where robot origin (0,0) is in field coords
    double              field_origin_y = 0.0   // where robot origin (0,0) is in field coords
);

// =========================================================================
// 9.  Internal helpers (in header for single-TU builds; move to .cpp if preferred)
// =========================================================================
Vec2   robot_to_field(double vr, double vf, double theta_deg);
Vec2   sensor_dir_field(double phi_deg, double theta_deg);
Vec2   sensor_origin_field(double x, double y, double theta_deg, const SensorConfig& sc);
double ray_segment_intersect(Vec2 origin, Vec2 dir, Vec2 a, Vec2 b, double t_max);
double noise_sigma(double z_in);
Vec2   closest_point_on_segment(Vec2 p, Vec2 a, Vec2 b);
double snap_median(double* arr, int n);

// ---------------------------------------------------------------------------
// snapshot_example.cpp  –  LemLib edition
// ---------------------------------------------------------------------------
// Drop this into your main.cpp (or #include it).  The only things you need to
// edit are marked with  ▶ EDIT ◀  below.
// ---------------------------------------------------------------------------

// =========================================================================
// ▶ EDIT ◀  1.  Declare your chassis (copy from your existing main.cpp)
// =========================================================================
// These are just stubs so the file compiles standalone.  Replace with your
// real declarations or make sure they are already visible in scope.
//
// extern lemlib::Chassis chassis;   // ← uncomment if chassis is declared elsewhere

// =========================================================================
// Sensor mount offsets  (inches from chassis centre)
// =========================================================================
//   offset_right:  + = right of centre
//   offset_fwd:    + = forward of centre
//
//            ┌───────────┐
//            │           │
//     C ←────│           │        C  = left          (phi = -90°)
//   (left)   │     ·     │        A  = back-left     (phi = 180°)
//            │           │        B  = back-right    (phi = 180°)
//            └───┬───┬───┘        ·  = chassis centre
//                A   B
//                ↓   ↓
//
//                   right     fwd      phi     port   mask
static const SensorConfig SENSORS[NUM_SENSORS] = {
/*  back-left  */  { -5.1875, -3.625,  180.0,    9,   0 },
/*  back-right */  {  5.1875, -3.625,  180.0,   21,   0 },
/*  left       */  { -4.9375,  4.25,   -90.0,   17,   0 },
};
// mask_override = 0  means "use the global mask passed at call time".
// If one sensor is mounted high enough to see long goals, change its
// mask_override to  MASK_PERIMETER | MASK_LONG_GOALS.

// =========================================================================
// Callback: read one sensor by port number
// =========================================================================
static SensorReading read_sensor_cb(int port) {
    pros::Distance* dev = nullptr;
    switch (port) {
        case  9:  dev = &back_left_sensor;   break;
        case 21:  dev = &back_right_sensor;  break;
        case 17:  dev = &left_sensor;        break;
        default:  return { -1, 0 };
    }
    int dist_mm = static_cast<int>(dev->get());
    int conf    = static_cast<int>(dev->get_confidence());
    if (dist_mm < 0) return { -1, 0 };
    return { dist_mm, conf };
}

// =========================================================================
// Callback: delay in ms  (PROS)
// =========================================================================
static void delay_cb(int ms) {
    pros::delay(ms);
}

// =========================================================================
// High-level helper  –  call this inside your auton
// =========================================================================
//
//   SnapshotResult res = do_snapshot(Quadrant::BL);
//   // res.ok == true  →  chassis pose has been updated
//   // res.ok == false →  nothing changed, odometry left alone
//
// OPTIONAL: For robot-centered odometry that needs conversion to field coords,
// pass the field position of your robot's starting point:
//   SnapshotResult res = do_snapshot(Quadrant::TR, 60.0, 60.0);
//   // This tells snapshot: "my (0,0) started at field position (60, 60)"
//
inline SnapshotResult do_snapshot(Quadrant quadrant, double field_origin_x = 0.0, double field_origin_y = 0.0) {
    // grab current pose from LemLib (degrees, robot-centered or field-centered)
    lemlib::Pose cur = chassis.getPose();   // .x  .y  .theta (degrees)

    SnapshotResult res = snapshot_setpose(
        SENSORS,
        DEFAULT_FIELD_MAP,
        DEFAULT_FIELD_MAP_SIZE,
        MASK_PERIMETER,          // global mask – walls only (safe default)
        cur.x,
        cur.y,
        cur.theta,               // degrees, 0° up, CW+ — matches LemLib
        quadrant,
        read_sensor_cb,
        delay_cb,
        field_origin_x,          // where robot (0,0) is in field coords
        field_origin_y
    );

    if (res.ok) {
        // Apply the corrected (x, y) while keeping the current heading.
        // Result is ALWAYS in field coordinates now
        // setPose second arg false = theta is in degrees (the default).
        chassis.setPose(lemlib::Pose(res.x_in, res.y_in, cur.theta));
    }

    return res;   // caller can inspect .ok, .chi2, .sensors_used if desired
}