#include "main.h"
// ── Auto Selector ─────────────────────────────────────────────────────────────
 
const char* AUTO_NAMES[] = {
    "Skills",
    "Red Close",
    "Red Far",
    "Blue Close",
    "Blue Far",
};
const int NUM_AUTOS = sizeof(AUTO_NAMES) / sizeof(AUTO_NAMES[0]);
int selectedAuto = 0;
 
// Global pointer so autonomous() can suspend the task before running
pros::Task* selectorTask = nullptr;
 
#define COL_BG       0x00111111
#define COL_ARROW_BG 0x00333333
#define COL_WHITE    0x00FFFFFF
#define COL_GREY     0x00888888
#define COL_ACCENT   0x000077FF
 
void drawSelector() {
    // Centre background — only fills between the two panels
    pros::screen::set_pen(COL_BG);
    pros::screen::fill_rect(80, 0, 400, 240);
 
    // Left arrow panel
    pros::screen::set_pen(COL_ARROW_BG);
    pros::screen::fill_rect(0, 0, 80, 240);
    for (int i = 0; i < 20; i++) {
        pros::screen::set_pen(COL_WHITE);
        pros::screen::draw_line(30 + i, 120 - i, 30 + i, 120 + i);
    }
 
    // Right arrow panel
    pros::screen::set_pen(COL_ARROW_BG);
    pros::screen::fill_rect(400, 0, 480, 240);
    for (int i = 0; i < 20; i++) {
        pros::screen::set_pen(COL_WHITE);
        pros::screen::draw_line(450 - i, 120 - i, 450 - i, 120 + i);
    }
 
    // Accent bars
    pros::screen::set_pen(COL_ACCENT);
    pros::screen::fill_rect(80, 0, 400, 6);
    pros::screen::fill_rect(80, 234, 400, 240);
 
    // Index text — clear row first so no eraser bleed
    pros::screen::set_pen(COL_BG);
    pros::screen::fill_rect(81, 40, 399, 85);
    pros::screen::set_pen(COL_GREY);
    pros::screen::print(pros::E_TEXT_MEDIUM, 195, 52,
        "%d / %d", selectedAuto + 1, NUM_AUTOS);
 
    // Auto name — clear row first
    pros::screen::set_pen(COL_BG);
    pros::screen::fill_rect(81, 86, 399, 160);
    pros::screen::set_pen(COL_WHITE);
    pros::screen::print(pros::E_TEXT_LARGE, 95, 105,
        "%s", AUTO_NAMES[selectedAuto]);
}
 
inline void autoSelectorTask(void*) {
    int lastAuto = -1;
    bool wasTouched = false;
 
    while (true) {
        if (selectedAuto != lastAuto) {
            drawSelector();
            lastAuto = selectedAuto;
        }
 
        pros::screen_touch_status_s_t touch = pros::screen::touch_status();
        bool isTouched = (touch.touch_status == pros::E_TOUCH_PRESSED ||
                          touch.touch_status == pros::E_TOUCH_HELD);
 
        if (isTouched && !wasTouched) {
            if (touch.x < 80) {
                selectedAuto = (selectedAuto - 1 + NUM_AUTOS) % NUM_AUTOS;
            } else if (touch.x > 400) {
                selectedAuto = (selectedAuto + 1) % NUM_AUTOS;
            }
        }
 
        wasTouched = isTouched;
        pros::delay(20);
    }
}