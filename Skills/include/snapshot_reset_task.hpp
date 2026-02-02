#pragma once
#include "pros/rtos.hpp"
#include "distanceReset/snapshot_bindings.hpp"

// -------------------------------------------------------
// Internal state - do not touch
// -------------------------------------------------------
namespace snapshot_reset_task {
    inline bool is_running       = false;
    inline bool request_pending  = false;   // NEW: flag to wake the persistent task
    inline snapshot_pose::SnapshotResult last_result;
    inline snapshot_pose::Quadrant pending_quadrant = snapshot_pose::Quadrant::AUTO;
    inline bool task_created     = false;   // NEW: only spawn the task once
}

// -------------------------------------------------------
// Persistent worker task - spawned ONCE, lives forever.
// Sleeps in a tight poll loop until request_pending is set.
// This completely avoids the old spawn/delete pattern that
// caused the kernel hang.
// -------------------------------------------------------
inline void snapshot_reset_task_fn(void*) {
    while (true) {
        // Sleep until someone sets request_pending
        if (!snapshot_reset_task::request_pending) {
            pros::delay(10);   // yield; don't busy-spin
            continue;
        }

        // Grab the request and clear the flag atomically-ish
        // (single writer from opcontrol, single reader here — safe on PROS)
        snapshot_reset_task::request_pending = false;
        snapshot_reset_task::is_running      = true;

        // Do the actual work (this is the slow part — ~90-525 ms)
        snapshot_reset_task::last_result = snapshot_pose::snapshot_setpose_quadrant(
            snapshot_reset_task::pending_quadrant
        );

        snapshot_reset_task::is_running = false;
    }
}

// -------------------------------------------------------
// Ensure the persistent task exists (call-once helper)
// -------------------------------------------------------
static inline void ensure_task_exists() {
    if (!snapshot_reset_task::task_created) {
        // Heap-allocate so it outlives this scope. We never delete it.
        new pros::Task(snapshot_reset_task_fn, nullptr, "SnapshotReset");
        snapshot_reset_task::task_created = true;
    }
}

// -------------------------------------------------------
// NON-BLOCKING: starts the reset, returns immediately.
// Use this in DRIVER CONTROL.
// Returns false if a reset is already running.
// -------------------------------------------------------
inline bool snapshot_reset_async(snapshot_pose::Quadrant quadrant = snapshot_pose::Quadrant::AUTO) {
    ensure_task_exists();

    if (snapshot_reset_task::is_running) return false;   // previous run still going

    snapshot_reset_task::pending_quadrant = quadrant;
    snapshot_reset_task::request_pending  = true;        // wake the worker

    return true;
}

// -------------------------------------------------------
// BLOCKING: starts the reset and waits for it to finish.
// Use this in AUTONOMOUS.
// Yields every 20 ms so your Coords task keeps running.
// -------------------------------------------------------
inline snapshot_pose::SnapshotResult snapshot_reset_blocking(snapshot_pose::Quadrant quadrant = snapshot_pose::Quadrant::AUTO) {
    snapshot_reset_async(quadrant);

    // Wait for the worker to pick it up AND finish
    // First wait for it to actually start (request_pending goes false)
    while (snapshot_reset_task::request_pending) {
        pros::delay(10);
    }
    // Then wait for it to finish
    while (snapshot_reset_task::is_running) {
        pros::delay(20);
    }

    return snapshot_reset_task::last_result;
}

// -------------------------------------------------------
// Check if a reset is still running
// -------------------------------------------------------
inline bool snapshot_reset_is_running() {
    return snapshot_reset_task::is_running;
}

// -------------------------------------------------------
// Get the result of the last reset that ran
// -------------------------------------------------------
inline snapshot_pose::SnapshotResult snapshot_reset_get_last_result() {
    return snapshot_reset_task::last_result;
}