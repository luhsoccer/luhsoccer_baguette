#pragma once

#include <utility>

#include "module.hpp"
#include "logger/logger.hpp"

namespace luhsoccer
{
namespace local_planner
{
class LocalPlannerModule;
}
namespace transform
{
class WorldModel;
}

namespace skills
{
class BodSkillBook;

class SkillTester : public BaguetteModule
{
   public:
    SkillTester(
        const BodSkillBook& skill_book,
        local_planner::LocalPlannerModule& local_planner_module,
        std::shared_ptr<const transform::WorldModel> wm)
        : skill_book(skill_book),
          local_planner_module(local_planner_module),
          wm(std::move(wm)),
          logger("SkillTester")
    {
    }

    void loop(std::atomic_bool& should_run) override;

    constexpr std::string_view moduleName() override { return "SkillTester"; }

    const BodSkillBook& skill_book;
    local_planner::LocalPlannerModule& local_planner_module;
    std::shared_ptr<const transform::WorldModel> wm;

    logger::Logger logger;
};
}  // namespace skills
}  // namespace luhsoccer