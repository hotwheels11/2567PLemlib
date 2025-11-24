#pragma once

struct MoveToPointParams {
    double maxSpeed = 100.0;
    double minSpeed = 20.0;
    double earlyExitRange = 2.0;
    bool forwards = true;
};

struct TurnToHeadingParams {
    double maxSpeed = 100.0;
    double minSpeed = 20.0;
};