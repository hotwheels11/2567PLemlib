#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "ExternalSystems.hpp" // IWYU pragma: keep

void line_Up_Auto(){
    chassis.moveToPoint(0,14,2000,{.maxSpeed = 40});
}

void right_Side() {
}

void left_Side() {
    
}