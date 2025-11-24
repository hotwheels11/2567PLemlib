#pragma once
#include "MoveParams.hpp"
#include "MCL/ParticleFilter.hpp"
#include "api.h"
#include <cmath>
#include <algorithm>

class MotionController {
private:
    ParticleFilter<500>* mcl;
    
    pros::MotorGroup* left_motors = nullptr;
    pros::MotorGroup* right_motors = nullptr;
    
    double slew_rate = 5.0;
    double current_left = 0.0;
    double current_right = 0.0;
    
    bool is_moving = false;
    float target_x = 0.0f;
    float target_y = 0.0f;
    float target_heading = 0.0f;
    uint32_t motion_start = 0;
    uint32_t timeout_ms = 0;
    
    float start_x = 0.0f;
    float start_y = 0.0f;
    float distance_moved = 0.0f;
    
    MoveToPointParams move_params;
    TurnToHeadingParams turn_params;
    
    enum MotionType { MOVE_TO_POINT, TURN_TO_HEADING, NONE } motion_type = NONE;
    
    double applySlew(double current, double target) {
        double diff = target - current;
        if (fabs(diff) <= slew_rate) return target;
        return current + (diff > 0 ? slew_rate : -slew_rate);
    }
    
    float normalizeAngle(float angle) {
        while (angle > M_PI) angle -= 2.0f * M_PI;
        while (angle < -M_PI) angle += 2.0f * M_PI;
        return angle;
    }
    
public:
    MotionController(ParticleFilter<500>* localization) : mcl(localization) {}
    
    void setMotors(pros::MotorGroup* left, pros::MotorGroup* right) {
        left_motors = left;
        right_motors = right;
    }
    
    void setSlewRate(double rate) {
        slew_rate = rate;
    }
    
    void moveToPoint(float x, float y, uint32_t timeout, MoveToPointParams params = {}) {
        target_x = x;
        target_y = y;
        timeout_ms = timeout;
        move_params = params;
        motion_start = pros::millis();
        start_x = mcl->getX();
        start_y = mcl->getY();
        distance_moved = 0.0f;
        is_moving = true;
        motion_type = MOVE_TO_POINT;
    }
    
    void turnToHeading(float heading_deg, uint32_t timeout, TurnToHeadingParams params = {}) {
        target_heading = heading_deg * M_PI / 180.0f;
        timeout_ms = timeout;
        turn_params = params;
        motion_start = pros::millis();
        distance_moved = 0.0f;
        is_moving = true;
        motion_type = TURN_TO_HEADING;
    }
    
    void cancelMotion() {
        is_moving = false;
        motion_type = NONE;
        if (left_motors && right_motors) {
            left_motors->move(0);
            right_motors->move(0);
        }
        current_left = 0.0;
        current_right = 0.0;
    }
    
    bool isMoving() const { return is_moving; }
    float getDistanceMoved() const { return distance_moved; }
    
    void waitUntil(float inches) {
        while (is_moving && distance_moved < inches) {
            pros::delay(10);
        }
    }
    
    void waitUntilDone() {
        while (is_moving) {
            pros::delay(10);
        }
    }
    
    void update() {
        if (!is_moving || !left_motors || !right_motors) return;
        
        if (pros::millis() - motion_start > timeout_ms) {
            cancelMotion();
            return;
        }
        
        float x = mcl->getX();
        float y = mcl->getY();
        float theta = mcl->getTheta();
        
        float dx = x - start_x;
        float dy = y - start_y;
        distance_moved = sqrt(dx * dx + dy * dy);
        
        double target_left = 0.0;
        double target_right = 0.0;
        
        if (motion_type == MOVE_TO_POINT) {
            float error_x = target_x - x;
            float error_y = target_y - y;
            float distance = sqrt(error_x * error_x + error_y * error_y);
            
            if (distance < move_params.earlyExitRange) {
                cancelMotion();
                return;
            }
            
            float desired_heading = atan2(error_y, error_x);
            if (!move_params.forwards) {
                desired_heading += M_PI;
            }
            
            float heading_error = normalizeAngle(desired_heading - theta);
            
            double forward = std::min(move_params.maxSpeed, 
                                     std::max(move_params.minSpeed, distance * 10.0));
            double turn = heading_error * 30.0;
            
            if (!move_params.forwards) {
                forward = -forward;
            }
            
            target_left = forward - turn;
            target_right = forward + turn;
            
        } else if (motion_type == TURN_TO_HEADING) {
            float heading_error = normalizeAngle(target_heading - theta);
            
            if (fabs(heading_error) < 0.05f) {
                cancelMotion();
                return;
            }
            
            double turn = std::min(turn_params.maxSpeed, 
                                  std::max(turn_params.minSpeed, fabs(heading_error) * 50.0));
            
            if (heading_error > 0) {
                target_left = -turn;
                target_right = turn;
            } else {
                target_left = turn;
                target_right = -turn;
            }
        }
        
        target_left = std::clamp(target_left, -127.0, 127.0);
        target_right = std::clamp(target_right, -127.0, 127.0);
        
        current_left = applySlew(current_left, target_left);
        current_right = applySlew(current_right, target_right);
        
        left_motors->move(current_left);
        right_motors->move(current_right);
    }
};