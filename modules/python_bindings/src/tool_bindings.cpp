#include "bindings.hpp"

#include "local_planner/skills/skill.hpp"
#include "utils/luts.hpp"

namespace luhsoccer::python {

void createToolBindings(py::module_& baguette_module, py::class_<baguette::TheBaguette>& wrapper,
                        baguette::TheBaguette& baguette) {
    // Here is the place to register more tools to the python bindings

    // Common bindings
    loadEnumBindings<Team>(baguette_module, "Team");
    loadEnumBindings<TeamColor>(baguette_module, "TeamColor");
    loadClassBindings<RobotIdentifier>(baguette_module, "RobotIdentifier");

    // Util bindings
    loadClassBindings<util::Lut1D>(baguette_module, "Lut1D");
    loadClassBindings<util::Lut2D>(baguette_module, "Lut2D");

    // Time bindings
    loadClassBindings<time::Duration>(baguette_module, "Duration");
    loadClassBindings<time::TimePoint>(baguette_module, "TimePoint");
    baguette_module.def("now", &time::now);

    // Transform bindings
    loadEnumBindings<transform::GameState>(baguette_module, "GameState");
    loadClassBindings<transform::Position>(baguette_module, "Position");
    loadClassBindings<transform::RobotData>(baguette_module, "RobotData");
    loadDerivedClassBindings<transform::AllyRobotData, transform::RobotData>(baguette_module, "AllyRobotData");
    loadClassBindings<transform::TransformHeader>(baguette_module, "TransformHeader");
    loadClassBindings<transform::Transform>(baguette_module, "Transform");
    loadClassBindings<transform::RobotHandle>(baguette_module, "RobotHandle");
    loadEnumBindings<transform::BallState>(baguette_module, "BallState");
    loadClassBindings<transform::BallInfo>(baguette_module, "BallInfo");
    loadSharedClassBindings<transform::WorldModel>(baguette_module, "WorldModel");

    // Skills bindings
    loadEnumBindings<skills::BodSkillNames>(baguette_module, "SkillName");
    loadClassBindings<local_planner::Skill>(baguette_module, "Skill");
    loadClassBindings<skills::BodSkillBook>(baguette_module, "SkillBook");
    wrapper.def_property_readonly(
        "skill_book", [&](py::object& /*self*/) { return &baguette.skill_book; }, py::return_value_policy::reference);

    // Config Provider
    bindConfigs(baguette_module);
}

}  // namespace luhsoccer::python
