#include "main.h"
#include "autos.hpp"
#include "ExternalSystems.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "autoSelector.hpp"
#define MVLIB_USE_SIMPLES
#include "mvlib/api.hpp"
#include "mvlib/Optional/lemlib.hpp"

// ─────────────────────────────────────────────────────────────────────────────

void initialize() {
    chassis.calibrate();
    pros::Task coordDisplayTask(Coords);
    selectorTask = new pros::Task(autoSelectorTask);


    auto& logger = mvlib::Logger::getInstance();
    mvlib::setOdom(&chassis);
    logger.setRobot({
        .leftDrivetrain  = &left_motors,
        .rightDrivetrain = &right_motors
    });
    logger.setRobot({
        .leftDrivetrain  = &left_motors,
        .rightDrivetrain = &right_motors
    }, true);
    logger.start();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // Stop the selector task so it can't interfere with autonomous
    if (selectorTask != nullptr) {
        selectorTask->suspend();
    }
 
    // Clear screen and show which auto is running
    pros::screen::set_pen(COL_BG);
    pros::screen::fill_rect(0, 0, 480, 240);
    pros::screen::set_pen(COL_WHITE);
    pros::screen::print(pros::E_TEXT_LARGE, 100, 100,
        "Running: %s", AUTO_NAMES[selectedAuto]);
 
    switch (selectedAuto) {
        case 0: autoSkills();    break;
        case 1: autoRedClose();  break;
        case 2: autoRedFar();    break;
        case 3: autoBlueClose(); break;
        case 4: autoBlueFar();   break;
        default: break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────

bool middleGoalState = false;
bool middleGoalManual = false;
inline int middleGoalDescoreState = 0;

void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    Descore.set_value(1);
    color_sensor.set_led_pwm(100);

    while (true) {
        int leftY  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        chassis.tank(leftY, rightY);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            descore++;
        }
        Descore.set_value(descore % 2 == 1);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            middleGoalDescoreState++;
        }
        middleGoalDescore.set_value(middleGoalDescoreState % 2 == 1);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            MatchloadMech++;
        }
        Matchload.set_value(MatchloadMech % 2 == 0);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            middleGoalManual = !middleGoalManual;
        }

        bool middleGoalCommand =
            middleGoalManual || master.get_digital(DIGITAL_L2);
        middleGoal.set_value(middleGoalCommand);

        if (master.get_digital(DIGITAL_R1)) {
            intakeFront.move(127);
        } else if (master.get_digital(DIGITAL_R2)) {
            intake.move(-127);
        } else if (master.get_digital(DIGITAL_L1)) {
            intake.move(127);
        } else if (master.get_digital(DIGITAL_L2)) {
            intakeFront.move(110);
        } else {
            intake.move(0);
        }

        pros::delay(20);
    }
}