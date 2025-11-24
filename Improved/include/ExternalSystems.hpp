#pragma once
#include "main.h" // IWYU pragma: keep

namespace Systems {
    // IMU
    inline pros::IMU imu(17);
    
    // Tracking wheels
    inline pros::Rotation horiz_encoder(16);
    inline pros::Rotation vert_encoder(1);
    
    // Drive motors
    inline pros::MotorGroup left_motors({-9, -2, -12});
    inline pros::MotorGroup right_motors({7, 6 , 13});
    
    // Intake
    inline pros::Motor intake(17);
    
    // Distance sensors (adjust ports as needed)
    inline pros::Distance front_distance1(3);
    inline pros::Distance front_distance2(10);

    inline pros::Distance left_distance1(5);
    inline pros::Distance left_distance2(8);

    inline pros::Distance right_distance1(2);
    inline pros::Distance right_distance2(9);

    inline pros::Distance back_distance1(14);
    inline pros::Distance back_distance2(15);
}

void trackingLoop(void* param);