#pragma once

#include <utility>

#include "robot_control/skills/skill.hpp"
#include "config/config_store.hpp"

namespace luhsoccer::skills {

#define SKILL_DEFS                 \
    using namespace robot_control; \
    using TD_Pos = ComponentPosition::TaskDataType;

template <typename T>
std::shared_ptr<const T> make(T&& c) {
    return std::make_shared<T>(std::forward<T>(c));
}

class SkillBuilder {
   public:
    SkillBuilder(const SkillBuilder&) = delete;
    SkillBuilder& operator=(const SkillBuilder&) = delete;
    SkillBuilder(SkillBuilder&&) = delete;
    SkillBuilder& operator=(SkillBuilder&&) = delete;
    virtual ~SkillBuilder() = default;

    const robot_control::Skill build(const config_provider::ConfigStore& cs) {
        this->buildImpl(cs);
        assert(skill.steps.size() > 0);
        return this->skill;
    }

   protected:
    SkillBuilder(const std::string& name, const std::vector<std::string>& related_robot = {},
                 const std::vector<std::string>& required_point = {},
                 const std::vector<std::string>& required_double = {},
                 const std::vector<std::string>& required_int = {}, const std::vector<std::string>& required_bool = {},
                 const std::vector<std::string>& required_string = {})
        : skill(name, related_robot, required_point, required_double, required_int, required_bool, required_string){};

    virtual void buildImpl(const config_provider::ConfigStore& cs) = 0;

    template <typename T>
    void addStep(T&& s) {
        static_assert(std::is_base_of<robot_control::AbstractStep, T>::value);
        this->skill.steps.push_back(std::make_shared<T>(std::forward<T>(s)));
    }

    void addStepPtr(const std::shared_ptr<const robot_control::AbstractStep>& s) { this->skill.steps.push_back(s); }

    void addSteps(const std::vector<std::shared_ptr<const robot_control::AbstractStep>>& steps) {
        for (const auto& step : steps) {
            this->skill.steps.push_back(step);
        }
    }

   private:
    robot_control::Skill skill;
};

}  // namespace luhsoccer::skills