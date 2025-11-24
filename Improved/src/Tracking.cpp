#include "ExternalSystems.hpp"
#include "main.h"
#include <cmath>

void trackingLoop(void* param) {
    float prev_vert = 0.0f;
    float prev_horiz = 0.0f;
    float prev_theta = 0.0f;
    
    // 2" omni wheel tracking wheels
    // Circumference = π * diameter = π * 2" = 6.283185 inches
    const float WHEEL_CIRCUMFERENCE = 2.0f * M_PI;
    
    // Assuming standard VEX rotation sensor (360 ticks per revolution)
    const float TICKS_PER_REV = 360.0f;
    
    while (true) {
        // Read tracking wheels and convert to inches
        // Formula: (ticks / ticks_per_rev) * circumference
        float vert_pos = (Systems::vert_encoder.get_position() / (TICKS_PER_REV * 100.0f)) * WHEEL_CIRCUMFERENCE;
        float horiz_pos = (Systems::horiz_encoder.get_position() / (TICKS_PER_REV * 100.0f)) * WHEEL_CIRCUMFERENCE;
        float imu_heading = Systems::imu.get_heading() * M_PI / 180.0f;
        
        // Calculate changes
        float d_vert = vert_pos - prev_vert;
        float d_horiz = horiz_pos - prev_horiz;
        float d_theta = imu_heading - prev_theta;
        
        // Normalize angle change
        while (d_theta > M_PI) d_theta -= 2.0f * M_PI;
        while (d_theta < -M_PI) d_theta += 2.0f * M_PI;
        
        // Convert to global frame
        float avg_theta = (imu_heading + prev_theta) / 2.0f;
        float dx = d_vert * cos(avg_theta) - d_horiz * sin(avg_theta);
        float dy = d_vert * sin(avg_theta) + d_horiz * cos(avg_theta);
        
        // Update MCL
        if (Robot::mcl) {
            Robot::mcl->update(dx, dy, d_theta, imu_heading);
        }
        
        // Update motion controller
        if (Robot::chassis) {
            Robot::chassis->update();
        }
        
        prev_vert = vert_pos;
        prev_horiz = horiz_pos;
        prev_theta = imu_heading;
        
        pros::delay(10);
    }
}