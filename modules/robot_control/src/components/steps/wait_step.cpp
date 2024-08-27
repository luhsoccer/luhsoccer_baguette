#include "robot_control/components/steps/wait_step.hpp"
#include "robot_control/components/component_data.hpp"

namespace luhsoccer::robot_control {

WaitStep::WaitStep(WaitDurationT, const DoubleComponentParam& duration_in_seconds)
    : duration_in_seconds(duration_in_seconds) {}

WaitStep::WaitStep(WaitBoolT, const BoolComponentParam& wait) : wait(wait) {}

WaitStep::WaitStep(WaitUntilTimePointT, const time::TimePoint& timepoint) : timepoint(timepoint) {}

WaitStep::WaitStep(const ComponentPosition& position, const DoubleComponentParam& distance_traveled,
                   const std::optional<DoubleComponentParam>& timeout)
    : position(position), distance_traveled(distance_traveled), timeout(timeout) {}

WaitStep::WaitStep(const ComponentPosition& position1, const ComponentPosition& position2,
                   const DoubleComponentParam& distance, const BoolComponentParam& shorter_than_distance,
                   const std::optional<DoubleComponentParam>& timeout)
    : position1(position1),
      position2(position2),
      distance(distance),
      shorter_than_distance(shorter_than_distance),
      timeout(timeout) {}

WaitStep::WaitStep(WaitForBallT, const BoolComponentParam& in_dribbler, const DoubleComponentParam& timeout)
    : wait_for_ball_in_dribbler(in_dribbler), timeout(timeout) {}

std::pair<AbstractStep::StepState, robot_interface::RobotCommand> WaitStep::calcCommandMessage(
    const ComponentData& comp_data) const {
    time::TimePoint time = comp_data.time;
    if (timeout.has_value()) {
        if (time == time::TimePoint(0)) time = time::now();
        auto start_timeout = this->getCookie<time::TimePoint>(comp_data.td, "start");

        if (!start_timeout.has_value()) {
            this->setCookie(comp_data.td, "start", time);
        } else {
            if (time - start_timeout.value() > time::Duration(timeout->val(comp_data))) {
                return {AbstractStep::StepState::FINISHED, robot_interface::RobotCommand()};
            }
        }
    }

    if (duration_in_seconds.has_value()) {
        if (time == time::TimePoint(0)) time = time::now();
        auto start = this->getCookie<time::TimePoint>(comp_data.td, "start");

        if (!start.has_value()) {
            this->setCookie(comp_data.td, "start", time);
            return {AbstractStep::StepState::RUNNING, robot_interface::RobotCommand()};
        } else {
            // LOG_INFO(logger::Logger("WaitStep"), "Starttime {}: {}", robot, start.value());

            if (time - start.value() > time::Duration(duration_in_seconds->val(comp_data))) {
                return {AbstractStep::StepState::FINISHED, robot_interface::RobotCommand()};
            } else {
                return {AbstractStep::StepState::RUNNING, robot_interface::RobotCommand()};
            }
        }
    } else if (wait.has_value()) {
        if (wait->val(comp_data)) {
            return {AbstractStep::StepState::RUNNING, robot_interface::RobotCommand()};
        } else {
            return {AbstractStep::StepState::FINISHED, robot_interface::RobotCommand()};
        }
    } else if (timepoint.has_value()) {
        if (time == time::TimePoint(0)) time = time::now();

        if (timepoint.value() > time) {
            return {AbstractStep::StepState::RUNNING, robot_interface::RobotCommand()};
        } else {
            return {AbstractStep::StepState::FINISHED, robot_interface::RobotCommand()};
        }
    } else if (position.has_value() && distance_traveled.has_value()) {
        auto start = this->getCookie<Eigen::Affine2d>(comp_data.td, "start_transform");
        if (start.has_value()) {
            auto current_transform =
                this->position->positionObject(comp_data).getCurrentPosition(comp_data.wm, "", time);
            if (current_transform.has_value()) {
                double distance_from_start =
                    (current_transform.value().translation() - start.value().translation()).norm();
                if (distance_from_start > distance_traveled->val(comp_data)) {
                    return {AbstractStep::StepState::FINISHED, robot_interface::RobotCommand()};
                } else {
                    return {AbstractStep::StepState::RUNNING, robot_interface::RobotCommand()};
                }
            }
        } else {
            auto start_transform = this->position->positionObject(comp_data).getCurrentPosition(comp_data.wm, "", time);
            if (start_transform.has_value()) {
                this->setCookie(comp_data.td, "start_transform", start_transform.value());
            }
        }
        return {AbstractStep::StepState::RUNNING, robot_interface::RobotCommand()};
    } else if (position1.has_value() && position2.has_value() && distance.has_value() &&
               shorter_than_distance.has_value()) {
        auto pos1 = this->position1->positionObject(comp_data).getCurrentPosition(comp_data.wm, "", time);
        auto pos2 = this->position2->positionObject(comp_data).getCurrentPosition(comp_data.wm, "", time);

        if (pos1.has_value() && pos2.has_value()) {
            double dist = (pos1.value().translation() - pos2.value().translation()).norm();
            if (shorter_than_distance->val(comp_data)) {
                if (dist < distance->val(comp_data)) {
                    return {AbstractStep::StepState::FINISHED, robot_interface::RobotCommand()};
                } else {
                    return {AbstractStep::StepState::RUNNING, robot_interface::RobotCommand()};
                }
            } else {
                if (dist > distance->val(comp_data)) {
                    return {AbstractStep::StepState::FINISHED, robot_interface::RobotCommand()};
                } else {
                    return {AbstractStep::StepState::RUNNING, robot_interface::RobotCommand()};
                }
            }
        }
        return {AbstractStep::StepState::RUNNING, robot_interface::RobotCommand()};
    } else if (this->wait_for_ball_in_dribbler.has_value()) {
        auto robot_data = comp_data.wm->getAllyRobotData(comp_data.robot, time);
        if (!robot_data.has_value()) return {AbstractStep::StepState::ERROR, robot_interface::RobotCommand()};
        if (robot_data->ball_in_dribbler == this->wait_for_ball_in_dribbler->val(comp_data)) {
            return {AbstractStep::StepState::FINISHED, robot_interface::RobotCommand()};
        } else {
            return {AbstractStep::StepState::RUNNING, robot_interface::RobotCommand()};
        }
    } else {
        throw std::runtime_error("WaitStep: No valid parameters set");
    }
}
}  // namespace luhsoccer::robot_control