#pragma once
#include <optional>

class Sensor {
public:
    virtual std::optional<double> p(float x, float y, float theta) = 0;
    virtual void update() = 0;
    virtual ~Sensor() = default;
};