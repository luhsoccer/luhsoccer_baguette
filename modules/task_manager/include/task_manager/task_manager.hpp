#pragma once
#include "logger/logger.hpp"
#include "robot_identifier.hpp"
#include "time/time.hpp"
#include "transform/world_model.hpp"
#include "module.hpp"
#include "local_planner/local_planner.hpp"
#include "local_planner/local_planner_module.hpp"

namespace luhsoccer::role_manager {
class RoleManager;
}

namespace luhsoccer::game_data_provider {
class GameDataProvider;
}

namespace luhsoccer::skills {
enum class BodSkillNames;
class BodSkillBook;
}  // namespace luhsoccer::skills

namespace luhsoccer::observer {
class Observer;
}

namespace luhsoccer::task_manager {

using TaskCallback =
    std::function<std::unordered_map<RobotIdentifier, std::pair<skills::BodSkillNames, local_planner::TaskData>>(
        const std::vector<transform::RobotHandle>, const std::shared_ptr<const observer::Observer>,
        const std::shared_ptr<const transform::WorldModel>)>;

class TaskManager : public BaguetteModule {
   public:
    TaskManager(role_manager::RoleManager&, const game_data_provider::GameDataProvider&, const skills::BodSkillBook&,
                local_planner::LocalPlannerModule&);
    TaskManager(const TaskManager&) = delete;
    TaskManager(TaskManager&&) = delete;
    TaskManager& operator=(const TaskManager&) = delete;
    TaskManager& operator=(TaskManager&&) = delete;
    virtual ~TaskManager() = default;

    std::string_view moduleName() override { return "task_manager"; }

    void loop(std::atomic_bool& should_run) override;
    void registerCallback(const std::string& role, const TaskCallback& callback);
    void stop() override;
    void updateSkill(RobotIdentifier robot, skills::BodSkillNames skill, local_planner::TaskData data);

   private:
    bool isTaskDataSame(const local_planner::TaskData& td1, const local_planner::TaskData& td2) const;

    role_manager::RoleManager& role_manager;
    const game_data_provider::GameDataProvider& game_data_provider;
    const skills::BodSkillBook& book;
    local_planner::LocalPlannerModule& local_planner;
    logger::Logger logger{"TaskManager"};
    std::mutex map_mutex;
    std::unordered_map<std::string, TaskCallback> task_callbacks;
    std::unordered_map<RobotIdentifier, skills::BodSkillNames> last_skill;
    std::unordered_map<RobotIdentifier, local_planner::TaskData> last_task_data;
    static inline constexpr double TASK_MANAGER_FREQUENCY = 10.0;  // @todo change via config
    time::Rate rate{TASK_MANAGER_FREQUENCY};
};

}  // namespace luhsoccer::task_manager

// Here is place for your own code