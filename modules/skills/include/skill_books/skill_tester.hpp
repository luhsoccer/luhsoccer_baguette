#pragma once

#include <utility>

#include "core/module.hpp"
#include "logger/logger.hpp"

namespace luhsoccer {
namespace robot_control {
class RobotControlModule;
}
namespace transform {
class WorldModel;
}

namespace skills {
class SkillLibrary;

class SkillTester : public BaguetteModule {
   public:
    SkillTester(const SkillLibrary& skill_lib, robot_control::RobotControlModule& robot_control_module,
                std::shared_ptr<const transform::WorldModel> wm)
        : skill_lib(skill_lib), robot_control_module(robot_control_module), wm(std::move(wm)), logger("SkillTester") {}

    void loop(std::atomic_bool& should_run) override;

    constexpr std::string_view moduleName() override { return "SkillTester"; }

    const SkillLibrary& skill_lib;
    robot_control::RobotControlModule& robot_control_module;
    std::shared_ptr<const transform::WorldModel> wm;

    logger::Logger logger;
};
}  // namespace skills
}  // namespace luhsoccer