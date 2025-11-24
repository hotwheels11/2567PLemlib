#pragma once
#include "MCL/ParticleFilter.hpp"
#include "MCL/Chassis/MotionController.hpp"
#include <memory>

namespace Robot {
    inline std::unique_ptr<ParticleFilter<500>> mcl;
    inline std::unique_ptr<MotionController> chassis;
}