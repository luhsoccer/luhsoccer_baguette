#pragma once

namespace luhsoccer::config_provider {
struct ConfigStore;
}  // namespace luhsoccer::config_provider

namespace luhsoccer::robot_control {
class AbstractStep;

std::vector<std::shared_ptr<const AbstractStep>> getGoToBallSteps(const config_provider::ConfigStore& cs);

}  // namespace luhsoccer::robot_control