#pragma once

// Here is place for your own code

#include "common_types.hpp"
#include <filesystem>

namespace luhsoccer {

inline double cropAngle(double angle) {
    if (angle > L_PI) {
        angle -= 2 * L_PI;
    }
    if (angle < -L_PI) {
        angle += 2 * L_PI;
    }
    return angle;
}

std::filesystem::path getBaguetteDirectory();

}  // namespace luhsoccer