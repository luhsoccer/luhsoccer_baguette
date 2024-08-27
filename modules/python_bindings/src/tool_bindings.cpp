#include "bindings.hpp"

#include "utils/luts.hpp"
#include "core/events.hpp"

namespace luhsoccer::python {

void createToolBindings(nb::module_& baguette_module, nb::class_<baguette::TheBaguette>& wrapper) {
    // Here is the place to register more tools to the python bindings

    loadEnumBindings<ssl_interface::SSLStage>(baguette_module, "GameStage");

    // Event system bindings

    loadClassBindings<logger::Logger>(baguette_module, "Logger");

    loadToolBindings<event_system::EventSystem>(baguette_module, "NativeEventSystem");

    wrapper.def_ro("native_event_system", &baguette::TheBaguette::event_system, nb::rv_policy::reference_internal);

    // Common bindings
    loadEnumBindings<Team>(baguette_module, "Team");
    loadEnumBindings<TeamColor>(baguette_module, "TeamColor");
    loadEnumBindings<Division>(baguette_module, "Division");
    loadClassBindings<RobotIdentifier>(baguette_module, "RobotIdentifier");
    loadDerivedClassBindings<StartEvent, event_system::Event>(baguette_module, "StartEvent");
    loadDerivedClassBindings<StopEvent, event_system::Event>(baguette_module, "StopEvent");

    // Util bindings
    loadClassBindings<util::Lut1D>(baguette_module, "Lut1D");
    loadClassBindings<util::Lut2D>(baguette_module, "Lut2D");

    // Time bindings
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
    loadClassBindings<transform::FieldData>(baguette_module, "FieldData");
    loadClassBindings<transform::WorldModel>(baguette_module, "WorldModel");

    // Skills bindings
    loadEnumBindings<skills::BodSkillNames>(baguette_module, "SkillName");
    loadEnumBindings<skills::BodSkillNames>(baguette_module, "BodSkillNames");
    loadEnumBindings<skills::GameSkillNames>(baguette_module, "GameSkillNames");
    loadEnumBindings<skills::TestSkillNames>(baguette_module, "TestSkillNames");

    loadClassBindings<robot_control::Skill>(baguette_module, "Skill");
    loadClassBindings<skills::BodSkillBook>(baguette_module, "SkillBook");
    loadClassBindings<skills::SkillLibrary>(baguette_module, "SkillLibrary");
    wrapper.def_prop_ro(
        "skill_book", [&](baguette::TheBaguette& self) { return &self.skill_lib.bod_book; },
        nb::rv_policy::reference_internal);

    wrapper.def_prop_ro(
        "skill_lib", [&](baguette::TheBaguette& self) { return &self.skill_lib; }, nb::rv_policy::reference_internal);

    // Config Provider
    bindConfigs(baguette_module);
}

}  // namespace luhsoccer::python
