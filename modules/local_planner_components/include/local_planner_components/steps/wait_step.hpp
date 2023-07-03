#pragma once

#include <utility>

#include "local_planner/skills/skill_util.hpp"
#include "local_planner/skills/abstract_step.hpp"

namespace luhsoccer::local_planner {

class WaitForBall_t {};
class WaitDuration_t {};
class WaitBool_t {};
class WaitUntilTimePoint_t {};

constexpr WaitForBall_t WAIT_FOR_BALL;
constexpr WaitDuration_t WAIT_DURATION;
constexpr WaitBool_t WAIT_BOOL;
constexpr WaitUntilTimePoint_t WAIT_UNTIL_TIME_POINT;

class WaitStep : public AbstractStep {
   public:
    /**
     * @brief Waits for a certain amount of time
     * @param duration_in_seconds
     */
    WaitStep(WaitDuration_t, const DoubleComponentParam& duration_in_seconds);

    /**
     * @brief Waits until the value of @p wait is set to false
     * @param wait
     */
    WaitStep(WaitBool_t, const BoolComponentParam& wait);

    /**
     * @brief Waits until the timepoint is reached
     * @param timepoint
     */
    WaitStep(WaitUntilTimePoint_t, const time::TimePoint& timepoint);

    /**
     * @brief Waits until the robot has travelled a certain distance or the timeout is reached
     * @param position
     * @param distance_travelled
     * @param timeout
     */
    WaitStep(const ComponentPosition& position, const DoubleComponentParam& distance_travelled,
             const std::optional<DoubleComponentParam>& timeout = std::nullopt);

    /**
     * @brief Waits until the distance between position1 and position2 is greater or smaller than distance (indicated by
     * shorter_than_distance))
     * @param position1
     * @param position2
     * @param distance
     * @param shorter_than_distance
     * @param timeout
     */
    WaitStep(const ComponentPosition& position1, const ComponentPosition& position2,
             const DoubleComponentParam& distance, const BoolComponentParam& shorter_than_distance,
             const std::optional<DoubleComponentParam>& timeout = std::nullopt);

    /**
     * @brief Waits until the ball is in the dribbler or the timeout is reached
     * @param in_dribbler if true, waits until ball is in dribbler else waits until ball is out of dribbler
     * @param timeout
     */
    WaitStep(WaitForBall_t, const BoolComponentParam& in_dribbler, const DoubleComponentParam& timeout);

    [[nodiscard]] std::pair<StepState, robot_interface::RobotCommand> calcCommandMessage(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        const std::shared_ptr<AvoidanceManager>& am, time::TimePoint time = time::TimePoint(0)) const override;

   private:
    // wait for a certain amount of time
    std::optional<DoubleComponentParam> duration_in_seconds;

    // wait until the value of wait is set to false
    std::optional<BoolComponentParam> wait;

    // wait until the timepoint is reached
    std::optional<time::TimePoint> timepoint;

    // wait until the robot has travelled a certain distance
    std::optional<ComponentPosition> position;
    std::optional<DoubleComponentParam> distance_travelled;

    // wait until the distance between position1 and position2 is greater or smaller than distance
    std::optional<ComponentPosition> position1;
    std::optional<ComponentPosition> position2;
    std::optional<DoubleComponentParam> distance;
    std::optional<BoolComponentParam> shorter_than_distance;

    std::optional<BoolComponentParam> wait_for_ball_in_dribbler;
    std::optional<DoubleComponentParam> timeout;
};

}  // namespace luhsoccer::local_planner