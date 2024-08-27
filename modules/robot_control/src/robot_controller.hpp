#pragma once
#include <utility>

#include "core/robot_identifier.hpp"

#include "robot_control/skills/task_data.hpp"
#include "robot_control/robot_controller_state.hpp"
#include "marker_adapter.hpp"

namespace luhsoccer::transform {
class WorldModel;
}

namespace luhsoccer::robot_interface {
struct RobotCommand;
}

namespace luhsoccer::marker {
class MarkerService;
}

namespace luhsoccer::event_system {
class EventSystem;
}

namespace luhsoccer::robot_control {
class Skill;
class CooperationModule;

class RobotController {
   public:
    RobotController(const std::shared_ptr<const transform::WorldModel>& wm, const MarkerAdapter& ma,
                    const RobotIdentifier& robot, CooperationModule& coop_module);

    bool startController(robot_interface::RobotInterface& robot_interface, event_system::EventSystem& event_system,
                         double controller_freq);

    bool startSimulation();

    bool setTask(const Skill* skill, const TaskData& td);

    const Skill* getSkill() const;

    bool updateTaskData(const TaskData& td);

    void setActive(bool online);

    bool cancelCurrentTask();

    [[nodiscard]] RobotControllerState getState() const { return this->state; }

    [[nodiscard]] const Skill* getCurrentSkill() const {
        const std::shared_lock read_lock(this->task_mtx);
        return this->skill;
    }

    [[nodiscard]] std::optional<TaskData> getCurrentTaskData() const {
        const std::shared_lock read_lock(this->task_mtx);
        return this->task_data;
    }

    std::pair<std::optional<robot_interface::RobotCommand>, RobotControllerState> calcCurrentCommandMessage(
        const time::TimePoint& time = time::TimePoint(0));

   private:
    void update();

    void checkIfRobotOnField();

    static robot_interface::RobotCommand getNullCmd();

    void sendCommand(const robot_interface::RobotCommand& robot_command) const;

    void stopRobot() const;

    void setSkillInfo(const std::string& skill_info, const Skill* skill = nullptr,
                      const std::optional<TaskData>& td = std::nullopt);

    void updateSkillText() const;

    std::atomic<RobotControllerState> state{RobotControllerState::CREATED};
    const Skill* skill{nullptr};
    std::optional<TaskData> task_data;
    size_t current_step{0};

    RobotIdentifier robot_id;
    std::shared_ptr<const transform::WorldModel> wm;

    mutable std::shared_mutex task_mtx;

    CooperationModule& coop_module;

    std::string current_skill_name;
    size_t current_skill_repetitions{0};
    std::string last_skill_name;
    size_t last_skill_repetitions{0};

    // modules
    MarkerAdapter ma;
    robot_interface::RobotInterface* robot_interface{nullptr};

    logger::Logger logger;
};

}  // namespace luhsoccer::robot_control