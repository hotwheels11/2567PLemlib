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
