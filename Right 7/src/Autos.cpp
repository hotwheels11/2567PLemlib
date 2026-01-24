#include "lemlib/chassis/chassis.hpp"
#include "main.h" // IWYU pragma: keep
#include "ExternalSystems.hpp" // IWYU pragma: keep

void calculatePoseAuto() {
    Matchload.set_value(1); //Activates to hold balls in place
    pros::delay(1000);
    calculates_robot_position(); // saves automatically in savedPose
    pros::delay(1000);
    Matchload.set_value(0); //Deactivates
}