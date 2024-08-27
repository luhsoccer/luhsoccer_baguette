#pragma once
#include "logger/logger.hpp"
#include "core/robot_identifier.hpp"
#include "time/time.hpp"
#include "transform/world_model.hpp"
#include "core/module.hpp"
#include "transform/handles.hpp"
#include "robot_control/skills/task_data.hpp"
#include <variant>
namespace luhsoccer::game_data_provider {
class GameDataProvider;
}

namespace luhsoccer::skills {
enum class BodSkillNames;
enum class GameSkillNames;
enum class TestSkillNames;
class SkillLibrary;

}  // namespace luhsoccer::skills

namespace luhsoccer::robot_control {
class RobotControlModule;
}
namespace luhsoccer::observer {
class Observer;
}

namespace luhsoccer::task_manager {
using SkillNames = std::variant<skills::BodSkillNames, skills::GameSkillNames, skills::TestSkillNames>;

using TaskCallback = std::function<std::unordered_map<RobotIdentifier, std::pair<SkillNames, robot_control::TaskData>>(
    const std::vector<transform::RobotHandle>, const std::shared_ptr<const observer::Observer>,
    const std::shared_ptr<const transform::WorldModel>)>;

class TaskManager : public BaguetteModule {
   public:
    TaskManager(const game_data_provider::GameDataProvider&, const skills::SkillLibrary&,
                robot_control::RobotControlModule&);
    TaskManager(const TaskManager&) = delete;
    TaskManager(TaskManager&&) = delete;
    TaskManager& operator=(const TaskManager&) = delete;
    TaskManager& operator=(TaskManager&&) = delete;
    virtual ~TaskManager() = default;

    std::string_view moduleName() override { return "task_manager"; }

    bool updateTask(SkillNames skill, robot_control::TaskData data, bool force = false);

    std::optional<SkillNames> getLastSkill(const RobotIdentifier& robot) const;

    std::optional<robot_control::TaskData> getLastTaskData(const RobotIdentifier& robot) const;

   private:
    bool isTaskDataSame(const robot_control::TaskData& td1, const robot_control::TaskData& td2) const;

    const game_data_provider::GameDataProvider& game_data_provider;
    const skills::SkillLibrary& skill_library;
    robot_control::RobotControlModule& robot_control;
    logger::Logger logger{"TaskManager"};
    std::mutex map_mutex;
    std::unordered_map<std::string, TaskCallback> task_callbacks;
    std::unordered_map<RobotIdentifier, SkillNames> last_skill;
    std::unordered_map<RobotIdentifier, robot_control::TaskData> last_task_data;
    static inline constexpr double TASK_MANAGER_FREQUENCY = 20.0;  // @todo change via config
    time::Rate rate{TASK_MANAGER_FREQUENCY};
};

}  // namespace luhsoccer::task_manager

// Here is place for your own code