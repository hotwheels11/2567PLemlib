#include "main.h"
#include <sstream>
#include <iomanip>

void displayLoop(void* param) {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    
    while (true) {
        if (Robot::mcl) {
            // Controller display
            std::stringstream ss;
            ss << std::fixed << std::setprecision(1)
               << "(" << Robot::mcl->getX() << ", "
               << Robot::mcl->getY() << ", "
               << (Robot::mcl->getTheta() * 180.0f / M_PI) << ")";
            master.set_text(0, 0, ss.str());
            
            // Brain screen display
            pros::lcd::print(0, "X: %.1f Y: %.1f", Robot::mcl->getX(), Robot::mcl->getY());
            pros::lcd::print(1, "Theta: %.1f", Robot::mcl->getTheta() * 180.0f / M_PI);        
            if (Robot::chassis) {
                pros::lcd::print(2, "Dist: %.1f", Robot::chassis->getDistanceMoved());
            }
        }
        
        pros::delay(50);
    }
}