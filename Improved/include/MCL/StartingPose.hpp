#pragma once

// Define all possible starting positions
enum class StartPosition {
    RED_LEFT,      // Red alliance, left corner
    RED_RIGHT,     // Red alliance, right corner
    BLUE_LEFT,     // Blue alliance, left corner
    BLUE_RIGHT,    // Blue alliance, right corner
    CUSTOM         // Custom position
};

// Store the selected starting position (change this based on where you place robot!)
inline StartPosition STARTING_POSITION = StartPosition::RED_LEFT;

// Helper function to get position bounds for each start location
inline void getStartingBounds(StartPosition pos, float& min_x, float& max_x, float& min_y, float& max_y) {
    switch(pos) {
        case StartPosition::RED_LEFT:
            min_x = 0.0f; max_x = -24.0f;
            min_y = -40.0f; max_y = -72.0f;
            break;
        case StartPosition::RED_RIGHT:
            min_x = 0.0f; max_x = 24.0f;
            min_y = -40.0f; max_y = -72.0f;
            break;
        case StartPosition::BLUE_LEFT:
            min_x = 0.0f; max_x = -24.0f;
            min_y = 40.0f; max_y = 72.0f;
            break;
        case StartPosition::BLUE_RIGHT:
            min_x = 0.0f; max_x = 24.0f;
            min_y = 40.0f; max_y = 72.0f;
            break;
        case StartPosition::CUSTOM:
            // Set custom bounds in main.cpp
            min_x = -20.0f; max_x = 20.0f;
            min_y = -20.0f; max_y = 20.0f;
            break;
    }
}