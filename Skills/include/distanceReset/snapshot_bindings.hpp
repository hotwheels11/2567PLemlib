#pragma once
#include "distanceReset/snapshot_pose.hpp"

namespace snapshot_pose {

// Quadrant specification for improved reliability
// This helps narrow down which corner of the field you're in
enum class Quadrant {
    BOTTOM_LEFT,   // X: 0-72,   Y: 0-72   (red alliance left)
    BOTTOM_RIGHT,  // X: 72-144, Y: 0-72   (red alliance right)
    TOP_LEFT,      // X: 0-72,   Y: 72-144 (blue alliance left)
    TOP_RIGHT,     // X: 72-144, Y: 72-144 (blue alliance right)
    AUTO           // Automatically determine from odometry guess
};

// Simple API - automatically determines quadrant from current odometry
SnapshotResult snapshot_setpose_simple();

// Advanced API - specify which quadrant you're in for better reliability
// Recommended for auton when you know your starting position
SnapshotResult snapshot_setpose_quadrant(Quadrant q);

} // namespace snapshot_pose