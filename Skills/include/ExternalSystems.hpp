// This is where we will put exetnal system definitions
#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"

inline pros::Controller master(pros::E_CONTROLLER_MASTER);

inline pros::MotorGroup left_motors({-3, -13, -11}, pros::MotorGearset::blue); // left motors on ports 3, -13, -11
inline pros::MotorGroup right_motors({7, 18 , 19}, pros::MotorGearset::blue); // right motors on ports -7, 18, 19
inline pros::Imu imu(12); // imu on port 12
inline pros::Motor LeftTest(-11);
inline pros::Rotation horizontal_encoder(-20);
inline pros::Rotation vertical_encoder(-2);

inline pros::Motor intakeFront(-9);
inline pros::Motor intakeTop(-1);
inline pros::MotorGroup intake({-9,-1});

inline pros::adi::DigitalOut Descore('B');
inline pros::adi::DigitalOut Matchload('A');
inline pros::adi::DigitalOut middleGoal('C');
inline pros::adi::DigitalOut middleGoalDescore('D');

inline pros::Distance left_sensor(4);
inline pros::Distance back_left_sensor(14);
inline pros::Distance back_right_sensor(17);

inline pros::Optical color_sensor(8);

//public Values
inline int defaultState = 0;
inline int descore = 0;
inline int MatchloadMech = 1;


inline lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -2.5);
// vertical tracking wheel
inline lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0);

// drivetrain settings
inline lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              10.7, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);// odometry settings

inline lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
inline lemlib::ControllerSettings lateral_controller(5, // proportional gain (kP) //10
                                              0, // integral gain (kI)
                                              20.1, // derivative gain (kD) //50
                                              .93, // anti windup 3
                                              2, // small error range, in inches 1
                                              200, // small error range timeout, in milliseconds 100
                                              4, // large error range, in inches 3
                                              500, // large error range timeout, in milliseconds 500
                                              20 // maximum acceleration (slew) 20
);
inline lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0.2, // integral gain (kI)
                                              12, // derivative gain (kD)
                                              4.65, // anti windup
                                              2, // small error range, in inches
                                              200, // small error range timeout, in milliseconds
                                              4, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
inline lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.02 // expo curve gain
);

// input curve for steer input during driver control
inline lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.02 // expo curve gain 1.019
);

// create the chassis
inline lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);

inline void Coords(void*){
    while(true){
        std::stringstream ss;
		ss << std::fixed << std::setprecision(1) // change 1 to however many decimals you want
		<< "(" << chassis.getPose().x << ", "
		<< chassis.getPose().y << ", "
		<< chassis.getPose().theta << ")";
		master.set_text(0, 0, ss.str());
        pros::delay(20);
    }
}

// Field size in inches
constexpr double FIELD_SIZE = 144.0;
// Offsets (in inches) from robot center
// +X = forward, +Y = right
struct SensorOffset {
    double x;
    double y;
};

inline SensorOffset front_offset = {1.25, -4.625};
inline SensorOffset right_offset = {4.5, 4.25};
inline SensorOffset back_offset  = {-5, -6.7};

// Store the robot's estimated position
struct Position {
    double x;
    double y;
    double heading; // degrees
};

inline double stableRead(pros::Distance& sensor, int samples = 10) {
    double sum = 0;
    int count = 0;
    for(int i = 0; i < samples; i++) {
        double mm = sensor.get_distance();
        if(mm > 0 && mm < 5000) { sum += mm; count++; }
        pros::delay(20);
    }
    return (count > 0) ? sum / count : NAN;
}

// Store the robot's last calculated pose for later use
inline Position savedPose = {NAN, NAN, 0.0};
/*
// Calculates the robot center position at startup using averaged sensor readings
inline Position calculates_robot_position(bool save = true) {
    
    // Take multiple samples for stability
    int samples = 10;
    double front_sum = 0, right_sum = 0, back_sum = 0;

    for(int i = 0; i < samples; i++) {
        double f = front_sensor.get_distance();
        double r = right_sensor.get_distance();
        double b = back_sensor.get_distance();

        if(f > 0 && f < 5000) front_sum += f;
        if(r > 0 && r < 5000) right_sum += r;
        if(b > 0 && b < 5000) back_sum += b;

        pros::delay(20);
    }

    double front_in = front_sum / samples / 25.4;
    double right_in = right_sum / samples / 25.4;
    double back_in  = back_sum / samples / 25.4;

    double heading_deg = 0.0; // fixed startup heading

    // Field center offset
    double field_center = FIELD_SIZE / 2.0;

    // Compute Y from front/back sensors (forward/backward movement)
    double y_est = NAN;
    if(!std::isnan(front_in) && !std::isnan(back_in)) {
        y_est = ((FIELD_SIZE - front_in - front_offset.x + back_in + back_offset.x) / 2.0) - field_center;
    } else if(!std::isnan(front_in)) {
        y_est = (FIELD_SIZE - front_in - front_offset.x) - field_center;
    } else if(!std::isnan(back_in)) {
        y_est = (back_in + back_offset.x) - field_center;
    }

    // Compute X from right sensor (left/right movement), flipped so positive X = toward center
    double x_est = !std::isnan(right_in) ? field_center - (right_in + right_offset.y) : NAN;

    Position pos = {x_est, y_est, heading_deg};

    // Save the position globally if requested
    if(save) savedPose = pos;

    return pos;
}

// Helper function to reset chassis to last saved pose
inline void setChassisToSavedPose() {
    if(!std::isnan(savedPose.x) && !std::isnan(savedPose.y)) {
        chassis.setPose(savedPose.x, savedPose.y, savedPose.heading);
    }
}

// Background task to print position periodically (useful for debugging)
inline void position_task(void*) {
    while (true) {
        // Also print raw sensor values so you can verify sensors and ports
        double front_mm = front_sensor.get_distance();
        double right_mm = right_sensor.get_distance();
        double back_mm  = back_sensor.get_distance();

        Position pos = calculates_robot_position();
        pros::lcd::print(0, "X: %.1f", std::isnan(pos.x) ? -1.0 : pos.x);
        pros::lcd::print(1, "Y: %.1f", std::isnan(pos.y) ? -1.0 : pos.y);
        pros::lcd::print(2, "Heading: %.1f°", pos.heading);
        pros::lcd::print(3, "raw mm f:%.0f r:%.0f b:%.0f", front_mm, right_mm, back_mm);
        pros::delay(200);
    }
}
    */

inline void intakeColorSort() {
    static bool rejecting = false;
    static uint32_t rejectStart = 0;

    intake.move(127);

    int hue = color_sensor.get_hue();
    bool red = (hue > 300 || hue < 60);
    bool blue = !red;

    // EDGE TRIGGER: blue just detected, start reject
    if (blue && !rejecting) {
        rejecting = true;
        rejectStart = pros::millis();
        middleGoal.set_value(1);
    }

    // Hold reject for 200 ms regardless of sensor visibility
    if (rejecting) {
        middleGoal.set_value(1);

        if (pros::millis() - rejectStart >= 200) {
            rejecting = false;
            middleGoal.set_value(0);
        }
    }
}
// Constants - MEASURE THESE CAREFULLY FOR YOUR ROBOT
// All measurements from robot CENTER in inches

// Left sensor (facing right in your diagram)
const double LEFT_SENSOR_X_OFFSET = -4.9375;  // + means sensor is to the RIGHT of center
const double LEFT_SENSOR_Y_OFFSET = 4.25;     // + means sensor is BEHIND center

// Back left sensor
const double BACK_LEFT_SENSOR_X_OFFSET = -5.1875;  // - means sensor is to the LEFT of center
const double BACK_LEFT_SENSOR_Y_OFFSET = -3.625;   // + means sensor is BEHIND center

// Back right sensor
const double BACK_RIGHT_SENSOR_X_OFFSET = 5.1875;  // + means sensor is to the RIGHT of center
const double BACK_RIGHT_SENSOR_Y_OFFSET = -3.625;  // + means sensor is BEHIND center

const double FIELD_WIDTH = 144.0;
const double FIELD_LENGTH = 144.0;

// Maximum distance we expect to see when near a corner (inches)
const double MAX_CORNER_DISTANCE = 30.0;

/**
 * @brief Calculate theta offset using two distance sensors
 */
inline double calculateThetaFromWall(double d_left, double d_right, double x_left, double x_right) {
    double sensor_separation = x_right - x_left;
    double delta_d = d_right - d_left;
    double theta = atan2(delta_d, sensor_separation);
    return theta;
}

/**
 * @brief Determine which direction each sensor group is facing based on robot heading
 * 
 * @param heading Robot heading in degrees (0 = north, 90 = east, etc.)
 * @param backSensorDirection Output: which wall back sensors are facing
 * @param sideSensorDirection Output: which wall side sensor is facing
 */
inline void determineSensorDirections(double heading, std::string& backSensorDirection, std::string& sideSensorDirection) {
    // Normalize heading to 0-360
    while (heading < 0) heading += 360;
    while (heading >= 360) heading -= 360;
    
    // Back sensors face opposite to robot heading
    // Side sensor (left) faces 90 degrees to the right of robot heading
    
    // Determine back sensor direction (within ±45 degrees)
    if (heading >= 315 || heading < 45) {
        backSensorDirection = "south";  // Robot facing north, back sensors face south
        sideSensorDirection = "east";   // Left sensor faces east (right)
    } else if (heading >= 45 && heading < 135) {
        backSensorDirection = "west";   // Robot facing east, back sensors face west
        sideSensorDirection = "south";  // Left sensor faces south
    } else if (heading >= 135 && heading < 225) {
        backSensorDirection = "north";  // Robot facing south, back sensors face north
        sideSensorDirection = "west";   // Left sensor faces west
    } else if (heading >= 225 && heading < 315) {
        backSensorDirection = "east";   // Robot facing west, back sensors face east
        sideSensorDirection = "north";  // Left sensor faces north
    }
}

/**
 * @brief Automatically reset robot position using distance sensors
 * 
 * This function:
 * 1. Gets current estimated heading from IMU
 * 2. Reads distance sensors to determine actual position
 * 3. Determines which direction sensors are facing based on heading
 * 4. Calculates corrected position using sensor readings
 * 5. Updates pose with corrected values
 * 
 * @param chassis Your LemLib chassis object
 * @return true if reset was successful, false otherwise
 */
inline bool autoDistanceReset(lemlib::Chassis& chassis) {
    // Get current estimated pose (we trust theta from IMU, not x/y from odometry)
    lemlib::Pose currentPose = chassis.getPose();
    
    // Get sensor readings (convert mm to inches)
    double side_dist = left_sensor.get() / 25.4;
    double back_left_dist = back_left_sensor.get() / 25.4;
    double back_right_dist = back_right_sensor.get() / 25.4;
    
    // Debug: Show what we're reading
    pros::lcd::print(7, "Raw sensors: S=%.1f BL=%.1f BR=%.1f", 
                     side_dist, back_left_dist, back_right_dist);
    pros::lcd::print(8, "Current theta: %.1f", currentPose.theta);
    
    // Validate sensor readings - check for reasonable values
    if (side_dist <= 0 || side_dist > 78.0 || 
        back_left_dist <= 0 || back_left_dist > 78.0 ||
        back_right_dist <= 0 || back_right_dist > 78.0) {
        pros::lcd::print(0, "FAIL: Bad sensor data");
        pros::lcd::print(1, "S:%.1f BL:%.1f BR:%.1f", side_dist, back_left_dist, back_right_dist);
        return false;
    }
    
    // Check if ALL sensors indicate we're near walls (not just odometry guess)
    if (side_dist > MAX_CORNER_DISTANCE || 
        back_left_dist > MAX_CORNER_DISTANCE || 
        back_right_dist > MAX_CORNER_DISTANCE) {
        pros::lcd::print(0, "FAIL: Not in corner");
        pros::lcd::print(1, "Sensors too far from walls");
        pros::lcd::print(2, "S:%.1f BL:%.1f BR:%.1f", side_dist, back_left_dist, back_right_dist);
        return false;
    }
    
    // Determine which direction sensors are facing based on IMU heading
    std::string backSensorWall, sideSensorWall;
    determineSensorDirections(currentPose.theta, backSensorWall, sideSensorWall);
    
    pros::lcd::print(6, "Sensors face: B=%s S=%s", backSensorWall.c_str(), sideSensorWall.c_str());
    
    // Calculate theta correction from back sensors
    double theta_offset_rad = calculateThetaFromWall(
        back_left_dist, back_right_dist,
        BACK_LEFT_SENSOR_X_OFFSET, BACK_RIGHT_SENSOR_X_OFFSET
    );
    
    double avg_back_dist = (back_left_dist + back_right_dist) / 2.0;
    double perpendicular_dist = avg_back_dist * cos(theta_offset_rad);
    
    double x = 0, y = 0, theta = 0;
    bool position_calculated = false;
    
    // Calculate corrected position based on which walls sensors are facing
    if (backSensorWall == "north") {
        // Back sensors measuring distance to north wall (Y = 144)
        // Robot is facing south (theta ≈ 180)
        theta = 180 + (theta_offset_rad * 180.0 / M_PI);
        
        // Y position: distance from north wall minus sensor offset
        y = FIELD_LENGTH - perpendicular_dist - fabs(BACK_LEFT_SENSOR_Y_OFFSET);
        
        // X position depends on which wall side sensor faces
        if (sideSensorWall == "east") {
            x = FIELD_WIDTH - side_dist - fabs(LEFT_SENSOR_Y_OFFSET);
        } else if (sideSensorWall == "west") {
            x = side_dist + fabs(LEFT_SENSOR_Y_OFFSET);
        }
        position_calculated = true;
    } 
    else if (backSensorWall == "south") {
        // Back sensors measuring distance to south wall (Y = 0)
        // Robot is facing north (theta ≈ 0)
        theta = 0 + (theta_offset_rad * 180.0 / M_PI);
        
        // Y position: distance from south wall plus sensor offset
        y = perpendicular_dist + fabs(BACK_LEFT_SENSOR_Y_OFFSET);
        
        // X position depends on which wall side sensor faces
        if (sideSensorWall == "east") {
            x = FIELD_WIDTH - side_dist - fabs(LEFT_SENSOR_Y_OFFSET);
        } else if (sideSensorWall == "west") {
            x = side_dist + fabs(LEFT_SENSOR_Y_OFFSET);
        }
        position_calculated = true;
    }
    else if (backSensorWall == "east") {
        // Back sensors measuring distance to east wall (X = 144)
        // Robot is facing west (theta ≈ 270)
        theta = 270 + (theta_offset_rad * 180.0 / M_PI);
        
        // X position: distance from east wall minus sensor offset
        x = FIELD_WIDTH - perpendicular_dist - fabs(BACK_LEFT_SENSOR_Y_OFFSET);
        
        // Y position depends on which wall side sensor faces
        if (sideSensorWall == "north") {
            y = FIELD_LENGTH - side_dist - fabs(LEFT_SENSOR_Y_OFFSET);
        } else if (sideSensorWall == "south") {
            y = side_dist + fabs(LEFT_SENSOR_Y_OFFSET);
        }
        position_calculated = true;
    }
    else if (backSensorWall == "west") {
        // Back sensors measuring distance to west wall (X = 0)
        // Robot is facing east (theta ≈ 90)
        theta = 90 + (theta_offset_rad * 180.0 / M_PI);
        
        // X position: distance from west wall plus sensor offset
        x = perpendicular_dist + fabs(BACK_LEFT_SENSOR_Y_OFFSET);
        
        // Y position depends on which wall side sensor faces
        if (sideSensorWall == "north") {
            y = FIELD_LENGTH - side_dist - fabs(LEFT_SENSOR_Y_OFFSET);
        } else if (sideSensorWall == "south") {
            y = side_dist + fabs(LEFT_SENSOR_Y_OFFSET);
        }
        position_calculated = true;
    }
    
    // Verify we actually calculated a position
    if (!position_calculated) {
        pros::lcd::print(0, "FAIL: Could not calculate position");
        pros::lcd::print(1, "Invalid sensor wall combo");
        return false;
    }
    
    // Verify calculated position is within field bounds
    if (x < 0 || x > FIELD_WIDTH || y < 0 || y > FIELD_LENGTH) {
        pros::lcd::print(0, "FAIL: Position out of bounds");
        pros::lcd::print(1, "Calc: X=%.1f Y=%.1f", x, y);
        return false;
    }
    
    // Calculate correction applied
    double dx = x - currentPose.x;
    double dy = y - currentPose.y;
    double dtheta = theta - currentPose.theta;
    
    // Set the corrected pose
    chassis.setPose(x, y, theta);
    
    // Success output
    pros::lcd::print(0, "Reset SUCCESS");
    pros::lcd::print(1, "New: X=%.1f Y=%.1f", x, y);
    pros::lcd::print(2, "Theta: %.1f deg", theta);
    pros::lcd::print(3, "Correction: dX=%.1f dY=%.1f", dx, dy);
    pros::lcd::print(4, "dTheta: %.1f deg", dtheta);
    pros::lcd::print(5, "Sensors: S=%.1f BL=%.1f BR=%.1f", side_dist, back_left_dist, back_right_dist);
    
    return true;
}