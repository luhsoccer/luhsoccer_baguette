#pragma once

#include "time/time.hpp"

namespace luhsoccer::config_provider {
class DoubleParamClass;
}

namespace luhsoccer::robot_control {

constexpr int SIZE_OF_ROBOT_STATE_VEC = 6;
constexpr int SIZE_OF_CMD_VECTOR = 3;

using RobotState = Eigen::Matrix<double, SIZE_OF_ROBOT_STATE_VEC, 1>;

RobotState convertToRobotState(const Eigen::Affine2d& pos, const Eigen::Vector3d& vel);

std::pair<Eigen::Affine2d, Eigen::Vector3d> convertFromRobotState(const RobotState& robot_state);
class RobotDynamic {
   public:
    const static int STATE_X = 0;
    const static int STATE_Y = 1;
    const static int STATE_T = 2;
    const static int STATE_VX = 3;
    const static int STATE_VY = 4;
    const static int STATE_VT = 5;

    explicit RobotDynamic();

    [[nodiscard]] RobotState applyAcceleration(const RobotState& old_state, Eigen::Vector3d acc,
                                               time::Duration duration) const;
    [[nodiscard]] RobotState applyRobotCommandVelocity(RobotState robot_state, const Eigen::Vector3d& velocity,
                                                       time::Duration duration) const;

    static double clampAcceleration(double force, double vel, double max_acc, double max_brk);

    static double clampVelocity(double velocity, double max, std::optional<double> min = std::nullopt);

   private:
    [[nodiscard]] std::pair<Eigen::Matrix<double, SIZE_OF_ROBOT_STATE_VEC, SIZE_OF_ROBOT_STATE_VEC>,
                            Eigen::Matrix<double, SIZE_OF_ROBOT_STATE_VEC, SIZE_OF_CMD_VECTOR>>
    getSystemMatrices(double step_duration) const;

    config_provider::DoubleParamClass& mass;
    config_provider::DoubleParamClass& friction;
    config_provider::DoubleParamClass& acc_k_p;
    config_provider::DoubleParamClass& acc_k_v;
    config_provider::DoubleParamClass& acc_scale;
    config_provider::DoubleParamClass& max_vel_x;
    config_provider::DoubleParamClass& max_vel_y;
    config_provider::DoubleParamClass& max_vel_theta;
    config_provider::DoubleParamClass& max_acc_x;
    config_provider::DoubleParamClass& max_acc_y;
    config_provider::DoubleParamClass& max_acc_theta;
    config_provider::DoubleParamClass& max_brk_x;
    config_provider::DoubleParamClass& max_brk_y;
    config_provider::DoubleParamClass& max_brk_theta;
};
}  // namespace luhsoccer::robot_control