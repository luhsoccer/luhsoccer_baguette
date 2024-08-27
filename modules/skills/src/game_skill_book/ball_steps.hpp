

#pragma once

#include "robot_control/components/component_util.hpp"
namespace luhsoccer::config_provider {
struct ConfigStore;
}  // namespace luhsoccer::config_provider

namespace luhsoccer::robot_control {
class AbstractStep;
class DriveStep;
constexpr double LINE_TRANS_TOLERANCE = 0.001;
constexpr double LINE_CUTOFF_LENGTH = 10.0;

BoolComponentParam getBallMoving(const config_provider::ConfigStore& cs);
ComponentPosition getFutureBallPosition(const config_provider::ConfigStore& cs);

std::shared_ptr<DriveStep> getGetBallStep(const config_provider::ConfigStore& cs);

std::shared_ptr<const AbstractStep> getWaitForShotStep(const config_provider::ConfigStore& cs);

std::shared_ptr<const AbstractStep> getPrepositionForInterceptStep(const config_provider::ConfigStore& cs,
                                                                   const ComponentPosition& heading_target);

}  // namespace luhsoccer::robot_control