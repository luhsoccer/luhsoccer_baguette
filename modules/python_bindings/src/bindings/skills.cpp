#include "bindings.hpp"

namespace luhsoccer::python {

using namespace skills;

template <>
void bindClass(nb::class_<robot_control::Skill>& instance) {
    instance.def("__repr__", [](const robot_control::Skill& object) { return object.name; });
}

template <>
void bindClass(nb::class_<BodSkillBook>& instance) {
    instance.def(
        "getSkill",
        [](BodSkillBook& self, const BodSkillNames& name) -> const robot_control::Skill& {
            const robot_control::Skill& skill = self.getSkill(name);
            return skill;
        },
        nb::rv_policy::reference);
}

template <>
void bindClass(nb::class_<SkillLibrary>& instance) {
    instance.def(
        "getSkill",
        [](const SkillLibrary& self, const SkillNames& name) -> const robot_control::Skill& {
            const robot_control::Skill& skill = self.getSkill(name);
            return skill;
        },
        nb::rv_policy::reference);
}

}  // namespace luhsoccer::python