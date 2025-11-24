#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    
    // ========================================
    // STEP 1: SET YOUR STARTING POSITION HERE
    // ========================================
    // Just change this line based on where you place the robot!
    STARTING_POSITION = StartPosition::RED_LEFT;  // ← CHANGE THIS
    
    // Options:
    // StartPosition::RED_LEFT    - Red alliance, left corner
    // StartPosition::RED_RIGHT   - Red alliance, right corner  
    // StartPosition::BLUE_LEFT   - Blue alliance, left corner
    // StartPosition::BLUE_RIGHT  - Blue alliance, right corner
    
    // Reset and calibrate IMU
    Systems::imu.reset();
    pros::lcd::print(0, "Calibrating IMU...");
    
    int time = pros::millis();
    while (Systems::imu.is_calibrating()) {
        pros::delay(10);
        if (pros::millis() - time > 3000) break;
    }
    
    // Initialize MCL
    Robot::mcl = std::make_unique<ParticleFilter<500>>();
    
    // === ADD OBSTACLES ===
    // Your specific ladder objects at field center
    DistanceSensor::addObstacle(0.0f, 0.0f, 22.5f, 5.4375f, 45.0f);
    DistanceSensor::addObstacle(0.0f, 0.0f, 22.6875f, 4.3125f, -45.0f);
    
    // Add distance sensors
    auto* front_sensor1 = new DistanceSensor(3, 10.875f, 7.0625f, 0.0f, 0.986f);        // ID: 0
	auto* front_sensor2 = new DistanceSensor(10, -10.875f, 7.0625f, 0.0f, 0.986f);       // ID: 1
    auto* left_sensor1 = new DistanceSensor(5, -5.75f, 4.0625f, M_PI / 2.0f, 0.987f);  // ID: 2
	auto* left_sensor2 = new DistanceSensor(8, -6.0f, -0.3125, M_PI / 2.0f, 0.987f);  // ID: 3
    auto* right_sensor1 = new DistanceSensor(2, 5.75f, 4.0625f, -M_PI / 2.0f, 0.980f);// ID: 4
	auto* right_sensor2 = new DistanceSensor(9, 6.0f, -2.0625f, -M_PI / 2.0f, 0.980f);// ID: 5
    auto* back_sensor1 = new DistanceSensor(9, -6.5f, -4.9375f, 0.0f, 0.979f);         // ID: 6
	auto* back_sensor2 = new DistanceSensor(15, 6.5f, -4.9375f, 0.0f, 0.979f);         // ID: 7
    
    Robot::mcl->addSensor(front_sensor1);
	Robot::mcl->addSensor(front_sensor2);
    Robot::mcl->addSensor(left_sensor1);
	Robot::mcl->addSensor(left_sensor2);
    Robot::mcl->addSensor(right_sensor1);
	Robot::mcl->addSensor(right_sensor2);
    Robot::mcl->addSensor(back_sensor1);
	Robot::mcl->addSensor(back_sensor2);
    
    // ========================================
    // STEP 2: AUTO-LOCALIZE (Robot calculates pose + heading from sensors)
    // ========================================
    float min_x, max_x, min_y, max_y;
    getStartingBounds(STARTING_POSITION, min_x, max_x, min_y, max_y);
    
    pros::lcd::print(0, "=== LOCALIZING ===");
    pros::lcd::print(1, "Reading sensors...");
    pros::delay(1000);
    
    // Calculate X, Y, and HEADING from distance sensors using GEOMETRY
    bool success = Robot::mcl->initialLocalizationWithHeading(
        min_x, max_x,  // X bounds based on starting position
        min_y, max_y   // Y bounds based on starting position
    );
    
    if (success) {
        // Sync IMU to the sensor-calculated heading
        float calculated_heading = Robot::mcl->getTheta();
        Systems::imu.set_heading(calculated_heading * 180.0f / M_PI);
        
        pros::lcd::clear();
        pros::lcd::print(0, "=== SUCCESS! ===");
        pros::lcd::print(1, "X: %.1f  Y: %.1f", 
                        Robot::mcl->getX(), 
                        Robot::mcl->getY());
        pros::lcd::print(2, "Heading: %.1f deg",
                        calculated_heading * 180.0f / M_PI);
        pros::lcd::print(3, "Ready for auto!");
    } else {
        pros::lcd::clear();
        pros::lcd::print(0, "=== FAILED ===");
        pros::lcd::print(1, "Check:");
        pros::lcd::print(2, "- Sensors plugged in?");
        pros::lcd::print(3, "- Robot in corner?");
        pros::lcd::print(4, "- Touching 2 walls?");
        // Fallback to center
        Robot::mcl->initNormal(0.0f, 0.0f, 0.0f, 12.0f, 12.0f, 0.2f);
    }
    
    pros::delay(3000);
    
    // Initialize chassis controller
    Robot::chassis = std::make_unique<MotionController>(Robot::mcl.get());
    Robot::chassis->setMotors(&Systems::left_motors, &Systems::right_motors);
    Robot::chassis->setSlewRate(5.0);
    
    // Start background tasks
    pros::Task tracking_task(trackingLoop, nullptr, "Tracking");
    pros::Task display_task(displayLoop, nullptr, "Display");
    
    pros::lcd::print(3, "Init Complete!");
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
	// Select and run your autonomous routine
    Auton::redLeft();  // Change this to your desired auton
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
 pros::Controller master(pros::E_CONTROLLER_MASTER);
    
    while (true) {
        // Arcade drive
        int left = master.get_analog(ANALOG_LEFT_Y);
        int right = master.get_analog(ANALOG_RIGHT_X);
        Systems::left_motors.move(left);
        Systems::right_motors.move(right);
        
        // Intake control
        if (master.get_digital(DIGITAL_R1)) {
            Systems::intake.move(127);
        } else if (master.get_digital(DIGITAL_R2)) {
            Systems::intake.move(-127);
        } else {
            Systems::intake.move(0);
        }
        
        pros::delay(20);
    }
}

/*
==================== VEX HIGH STAKES FIELD REFERENCE ====================

FIELD LAYOUT (View from above, standing behind RED alliance):
==============================================================

                    BLUE ALLIANCE WALL
                         +Y (72")
    ┌─────────────────────────────────────────────┐
    │                                             │
    │  Blue          [LADDER]         Blue        │
    │  Corner          (0,0)          Corner      │
    │  (-60,60)                        (60,60)    │
-X  │                                             │  +X
(-72")                                           (72")
    │                                             │
    │  Red           [LADDER]          Red        │
    │  Corner          (0,0)           Corner     │
    │  (-60,-60)                       (60,-60)   │
    │                                             │
    └─────────────────────────────────────────────┘
                         -Y (-72")
                    RED ALLIANCE WALL


ALLIANCE STARTING ZONES:
=========================

RED ALLIANCE (Bottom):
- Left start: x ≈ -60, y ≈ -60
- Right start: x ≈ 60, y ≈ -60
- Touching bottom wall (Y = -72")

BLUE ALLIANCE (Top):
- Left start: x ≈ -60, y ≈ 60  
- Right start: x ≈ 60, y ≈ 60
- Touching top wall (Y = +72")


HEADING REFERENCE (0° points right):
====================================
        90° (UP/NORTH)
             |
             |
180° --------+-------- 0° (RIGHT/EAST)
(LEFT/WEST)  |
             |
        270° (DOWN/SOUTH)

For RED alliance facing field:
- Robot pointing up field (toward blue) = 90°
- Robot pointing right = 0°
- Robot pointing down field (toward red wall) = 270° or -90°

For BLUE alliance facing field:
- Robot pointing down field (toward red) = 270° or -90°
- Robot pointing right = 0°
- Robot pointing up field (toward blue wall) = 90°


TYPICAL STARTING CONFIGURATIONS:
=================================

RED ALLIANCE - LEFT START (touching corner):
Robot::mcl->initialLocalizationRegion(imu_heading,
    -65.0f, -50.0f,  // X: Left wall area
    -65.0f, -50.0f   // Y: Bottom wall area (RED side)
);
Expected heading: 0° to 90° (pointing into field/toward blue)

RED ALLIANCE - RIGHT START (touching corner):
Robot::mcl->initialLocalizationRegion(imu_heading,
     50.0f,  65.0f,  // X: Right wall area
    -65.0f, -50.0f   // Y: Bottom wall area (RED side)
);
Expected heading: 90° to 180° (pointing into field/toward blue)


BLUE ALLIANCE - LEFT START (touching corner):
Robot::mcl->initialLocalizationRegion(imu_heading,
    -65.0f, -50.0f,  // X: Left wall area
     50.0f,  65.0f   // Y: Top wall area (BLUE side)
);
Expected heading: 270° to 360° (pointing into field/toward red)

BLUE ALLIANCE - RIGHT START (touching corner):
Robot::mcl->initialLocalizationRegion(imu_heading,
     50.0f,  65.0f,  // X: Right wall area
     50.0f,  65.0f   // Y: Top wall area (BLUE side)
);
Expected heading: 180° to 270° (pointing into field/toward red)


LOCALIZATION WITH LIMITED SENSORS:
===================================

Corner start (2 walls visible):
- Sensor 1: Sees close wall (~3-5") ✅
- Sensor 2: Sees close wall (~3-5") ✅
- Sensor 3: May see ladder obstacle (~40-50") ✅
- Sensor 4: Times out (>80") ❌

Result: 2-3 valid readings = PERFECT for localization!

Mid-wall start (1 wall visible):
- Sensor 1: Sees close wall (~3-5") ✅
- Sensor 2: May see side wall (~60") ✅
- Sensor 3: May see ladder (~20-30") ✅
- Sensor 4: Times out ❌

Result: 2-3 valid readings = Still works!


IMPORTANT NOTES:
================
1. Distance sensors give X, Y position only
2. IMU gives heading (theta) separately  
3. Both are combined in MCL for full pose (x, y, theta)
4. Regional search only needs 2 sensors in range
5. Sensors that timeout (>80") are automatically ignored

*/

/*
==================== OBSTACLE SETUP GUIDE (WITH ROTATION) ====================

You have 3 options for initial localization:

METHOD 1: FULL FIELD SEARCH (Slowest, ~2 seconds)
==================================================
Use when starting position is completely unknown.

Robot::mcl->initialLocalization(imu_heading);

- Searches entire 144"x144" field
- Needs 3+ valid sensor readings
- Takes ~2 seconds


METHOD 2: REGIONAL SEARCH (RECOMMENDED - Fast, ~0.3 seconds)
=============================================================
Use when you know approximately where you start (like touching alliance wall).

Robot::mcl->initialLocalizationRegion(imu_heading, min_x, max_x, min_y, max_y);

Examples:

// Red alliance, left side, touching wall
Robot::mcl->initialLocalizationRegion(imu_heading, 
    -65.0f, -45.0f,  // X: 10-20 inches from left wall
     35.0f,  55.0f   // Y: upper field
);

// Blue alliance, right side, touching wall  
Robot::mcl->initialLocalizationRegion(imu_heading,
     45.0f,  65.0f,  // X: 10-20 inches from right wall
     35.0f,  55.0f   // Y: upper field
);

// Starting at bottom-left corner
Robot::mcl->initialLocalizationRegion(imu_heading,
    -65.0f, -45.0f,  // X: left wall area
    -55.0f, -35.0f   // Y: bottom wall area
);

// Starting at top-right corner
Robot::mcl->initialLocalizationRegion(imu_heading,
     45.0f,  65.0f,  // X: right wall area
     35.0f,  55.0f   // Y: top wall area
);

Benefits:
- Only needs 2+ valid sensor readings
- ~7x faster than full field search
- More reliable (smaller search space)
- Perfect for alliance-specific starting positions


METHOD 3: EXACT POSITION (Instant)
===================================
Use when you measure exact starting position with tape measure.

Robot::mcl->initNormal(x, y, imu_heading, std_x, std_y, std_theta);

Example:
Robot::mcl->initNormal(-55.0f, 45.0f, imu_heading, 2.0f, 2.0f, 0.1f);
//                      x      y       heading     uncertainty values

Benefits:
- Instant initialization
- No sensor readings needed
- Most accurate if you measure carefully


FIELD COORDINATE REFERENCE:
===========================
      +Y (72")
         |
   -X ---+--- +X
  (-72") |   (72")
         |
      -Y (-72")

Common VEX starting positions:
- Top-left corner: (-60, 60)
- Top-right corner: (60, 60)
- Bottom-left corner: (-60, -60)
- Bottom-right corner: (60, -60)
- Left wall, mid-field: (-60, 0)
- Right wall, mid-field: (60, 0)


SENSOR RANGE LIMITATIONS:
==========================
VEX distance sensors typically read 20-2000mm (0.8" - 78.7")

If starting position has:
- ✅ 2+ sensors reading walls/obstacles within range → Regional search works
- ✅ 3+ sensors reading within range → Full field search works
- ❌ Less than 2 sensors in range → Use exact position (Method 3)

For corner starts touching 2 walls:
- Close wall: ~2-6 inches (sensors read this)
- Far wall: ~132-138 inches (too far, sensors timeout)
- Obstacles: Depends on position
- Result: Usually 2-3 valid readings = perfect for regional search!


TIPS:
=====
1. Use regional search for autonomous (fast + reliable)
2. Adjust region based on robot size and starting rules
3. Make region ±10-15 inches around expected position
4. Use exact position for testing/debugging only

Now obstacles can be ROTATED at any angle!

SENSOR IDs (assigned automatically in order):
- Front sensor: ID 0
- Left sensor: ID 1
- Right sensor: ID 2
- Back sensor: ID 3

OBSTACLE SYNTAX:
DistanceSensor::addObstacle(x, y, width, height, rotation_degrees, {sensor_ids});


EXAMPLES:
=========

1. YOUR SPECIFIC OBJECTS (45° rotated at field center):

   // First object: 45° rotation
   DistanceSensor::addObstacle(0.0f, 0.0f, 22.5f, 5.4375f, 45.0f);
   
   // Second object: -45° rotation (perpendicular to first)
   DistanceSensor::addObstacle(0.0f, 0.0f, 22.6875f, 4.3125f, -45.0f);
   
   This creates an X-shaped obstacle at the field center!


2. NON-ROTATED OBSTACLE (0° = aligned with field):
   DistanceSensor::addObstacle(30.0f, 30.0f, 20.0f, 20.0f, 0.0f);
   // rotation = 0 means width is along X-axis, height along Y-axis


3. 90° ROTATED OBSTACLE:
   DistanceSensor::addObstacle(0.0f, 40.0f, 30.0f, 10.0f, 90.0f);
   // Now width (30") is along Y-axis, height (10") along X-axis


4. WITH SENSOR VISIBILITY:
   DistanceSensor::addObstacle(0.0f, 0.0f, 22.5f, 5.4375f, 45.0f, {0, 3});
   // 45° rotated, only visible to front (0) and back (3) sensors


5. MULTIPLE ROTATED OBSTACLES:
   // Four obstacles forming a + shape
   DistanceSensor::addObstacle(0.0f, 0.0f, 40.0f, 5.0f, 0.0f);    // Horizontal
   DistanceSensor::addObstacle(0.0f, 0.0f, 40.0f, 5.0f, 90.0f);   // Vertical
   
   // Four obstacles forming an X shape
   DistanceSensor::addObstacle(0.0f, 0.0f, 40.0f, 5.0f, 45.0f);   // Diagonal /
   DistanceSensor::addObstacle(0.0f, 0.0f, 40.0f, 5.0f, -45.0f);  // Diagonal \


ROTATION ANGLES:
================
    0° = Width along X-axis (right), Height along Y-axis (up)
   45° = Rotated counter-clockwise 45°
   90° = Width along Y-axis (up), Height along X-axis (right)
  -45° = Rotated clockwise 45° (same as 315°)

Visual example of rotation:
     0°: ═══════     45°:   ╱╱╱     90°: ║
                            ╱╱╱           ║
                           ╱╱╱            ║


COORDINATE SYSTEM:
==================
      +Y (72")
         |
   -X ---+--- +X
  (-72") |   (72")
         |
      -Y (-72")


REAL-WORLD VEX HIGH STAKES EXAMPLE:
====================================

// Center ladder (your 2 obstacles at 45° and -45°)
DistanceSensor::addObstacle(0.0f, 0.0f, 22.5f, 5.4375f, 45.0f);
DistanceSensor::addObstacle(0.0f, 0.0f, 22.6875f, 4.3125f, -45.0f);

// Mobile goals (not rotated, square)
DistanceSensor::addObstacle(24.0f, 24.0f, 13.0f, 13.0f, 0.0f, {0, 1, 2, 3});

// Wall stakes (tall, all sensors see)
DistanceSensor::addObstacle(0.0f, 72.0f, 6.0f, 6.0f, 0.0f);
DistanceSensor::addObstacle(72.0f, 0.0f, 6.0f, 6.0f, 0.0f);

// Rings on ground (only low sensors)
DistanceSensor::addObstacle(-40.0f, 0.0f, 4.0f, 4.0f, 0.0f, {1, 2});


PARAMETER BREAKDOWN:
====================
addObstacle(
    0.0f,        // center_x (inches from field center)
    0.0f,        // center_y (inches from field center)
    22.5f,       // width (inches along rotated X-axis)
    5.4375f,     // height (inches along rotated Y-axis)
    45.0f,       // rotation (degrees counter-clockwise)
    {0, 1, 2, 3} // visible_to_sensors (empty = all sensors)
);


TIPS:
=====
- Width/Height are measured BEFORE rotation
- Positive angles = counter-clockwise
- Negative angles = clockwise
- Leave sensor list empty if all sensors should detect it
- Use getSensorId() if you need to check a sensor's ID

*/