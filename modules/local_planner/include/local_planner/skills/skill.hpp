#pragma once

#include "marker_service/marker.hpp"
#include "robot_interface/robot_interface_types.hpp"
#include "logger/logger.hpp"
#include "abstract_component.hpp"
namespace luhsoccer::local_planner {
class AbstractStep;
class AvoidanceManager;
struct TaskData;
/**
 * @brief A skill that every robot is capable to execute.
 * A skill defines a behavior of a given robot. This behavior depends on the current world model and eventually on given
 * positions of constants. Examples for skills are 'getBall' or 'driveToPoint'. Different to the components is every
 * skill defined by a const object of type @class. The local planners execute skills with a const reference on a
 * particular skill object. Skill objects should not be copied or moved. All required runtime data has to be provided at
 * the function call.
 */
class Skill {
   public:
    /**
     * @brief Construct a new Skill object
     *
     * @param name name of the skill
     * @param related_robot number of related robots
     * @param required_point number of required points
     * @param required_double number of required doubles
     * @param required_int number of required ints
     * @param required_bool number of required bools
     * @param required_string number of required strings
     */
    Skill(std::string name, const std::vector<std::string>& related_robot,
          const std::vector<std::string>& required_point, const std::vector<std::string>& required_double,
          const std::vector<std::string>& required_int, const std::vector<std::string>& required_bool,
          const std::vector<std::string>& required_string);

    /// @brief state after a time step of a skill
    enum class SkillState {
        RUNNING,    ///< skill is in the normal operational state
        NEXT_STEP,  ///< the next step should be executed
        FINISHED,   ///< the skill is finished
        ERROR,      ///< an error occurred
        ABORT       ///< the skill was aborted from outside
    };
    /**
     * @brief calculate the current command message
     *
     * @param wm WorldModel
     * @param td TaskData
     * @param robot robot
     * @param time time point
     * @param step_num index of the current step
     * @return std::pair<State, robot_interface::RobotCommand> the state of the skill, the current command
     * message
     */
    [[nodiscard]] std::pair<SkillState, robot_interface::RobotCommand> calcCommandMessage(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        const time::TimePoint& time, const size_t step_num, const std::shared_ptr<AvoidanceManager>& am) const;

    /**
     * @brief Get the current visualization markers
     *
     * @param wm WorldModel
     * @param td TaskData
     * @param robot robot
     * @param time time point
     * @param step_num index of the current step
     * @return std::vector<Marker> list of the current visualization markers
     */
    [[nodiscard]] std::vector<Marker> getVisualizationMarkers(const std::shared_ptr<const transform::WorldModel>& wm,
                                                              const TaskData& td, const RobotIdentifier& robot,
                                                              const time::TimePoint& time, const size_t step_num) const;

    /**
     * @brief check if the task data is valid for this skill
     *
     * @param td TaskData
     * @return whether the given TaskData is valid for this skill
     */
    [[nodiscard]] bool taskDataValid(const TaskData& td) const;

    const std::string name;
    const size_t related_robot_num;
    const size_t required_point_num;
    const size_t required_double_num;
    const size_t required_int_num;
    const size_t required_bool_num;
    const size_t required_string_num;
    const std::vector<std::string> related_robot;
    const std::vector<std::string> required_point;
    const std::vector<std::string> required_double;
    const std::vector<std::string> required_int;
    const std::vector<std::string> required_bool;
    const std::vector<std::string> required_string;

    /// @brief the list of steps of this skill
    std::vector<std::shared_ptr<const AbstractStep>> steps;
};
}  // namespace luhsoccer::local_planner