// This is where we will put exetnal system definitions
#include "main.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/rtos.hpp"

inline pros::Controller master(pros::E_CONTROLLER_MASTER);

inline pros::MotorGroup left_motors({-9, -2, -12}, pros::MotorGearset::blue); // left motors on ports -9, -2, -12
inline pros::MotorGroup right_motors({7, 6 , 13}, pros::MotorGearset::blue); // right motors on ports 7, 6, 13
inline pros::Imu imu(17);// imu on port 10

inline pros::Rotation horizontal_encoder(16);
inline pros::Rotation vertical_encoder(1);

inline pros::Motor intakeFront(11);
inline pros::Motor intakemiddle(-18);
inline pros::Motor intakelate(-20);
inline pros::MotorGroup intake({11,-18,-20});
//A,F,G,H
inline pros::adi::DigitalOut Descore('A');
inline pros::adi::DigitalOut Park('G');
inline pros::adi::DigitalOut Matchload('H');
inline pros::adi::DigitalOut Hood('F');

inline pros::Distance ParkSensor(4);
inline pros::Distance front_sensor(10);
inline pros::Distance right_sensor(19);
inline pros::Distance back_sensor(14);


inline lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -0.3);
// vertical tracking wheel
inline lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 1.4);

// drivetrain settings
inline lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
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
inline lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              50, // derivative gain (kD)
                                              3, // anti windup 3
                                              0, // small error range, in inches 1
                                              0, // small error range timeout, in milliseconds 100
                                              0, // large error range, in inches 3
                                              0, // large error range timeout, in milliseconds 500
                                              20 // maximum acceleration (slew) 20
);

// angular PID controller
inline lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
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

inline bool buttonEnabled = false;
inline bool inPosition = false; 
// Background task that waits before enabling the button
inline void enableButtonTask(void*) {
    //pros::delay(70000); // Wait 70 seconds (20 seconds left in match)
    pros::delay(1000);
    buttonEnabled = true;
}

inline void activePosition(void*){
    while (true) {
        inPosition = (ParkSensor.get_distance() <= 20);  // true if ≤20, false otherwise
        pros::delay(100);
    }
}

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

//the log of change without moving the robot//
  /*(64.3,12.2,0)
    (64.3,12.4,0)
    (64.3,12.4,0)*/

    //(64.3,12.4,0)->(46.4,10,0)--->Restart program latest pose--->(46.4,9.9,0)-->Restart program from this pose-->(47.4,9,9,0)x2