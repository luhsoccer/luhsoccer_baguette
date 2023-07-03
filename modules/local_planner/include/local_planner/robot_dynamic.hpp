

#pragma once

#include <Eigen/Geometry>
#include "config/config_store.hpp"
#include "local_planner/local_planner_util.hpp"
#include "transform/transform.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::local_planner {

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

    explicit RobotDynamic(DoubleLocalPlannerParam mass = localPlannerConfig().simulation_mass,
                          DoubleLocalPlannerParam friction = localPlannerConfig().simulation_friction,
                          DoubleLocalPlannerParam acc_k_p = localPlannerConfig().simulation_acc_k_p,
                          DoubleLocalPlannerParam acc_k_v = localPlannerConfig().simulation_acc_k_v,
                          DoubleLocalPlannerParam acc_scale = localPlannerConfig().simulation_acc_scale,
                          DoubleLocalPlannerParam max_vel_x = localPlannerConfig().robot_vel_max_x,
                          DoubleLocalPlannerParam max_vel_y = localPlannerConfig().robot_vel_max_y,
                          DoubleLocalPlannerParam max_vel_theta = localPlannerConfig().robot_vel_max_theta,
                          DoubleLocalPlannerParam max_acc_x = localPlannerConfig().robot_acc_max_x,
                          DoubleLocalPlannerParam max_acc_y = localPlannerConfig().robot_acc_max_y,
                          DoubleLocalPlannerParam max_acc_theta = localPlannerConfig().robot_acc_max_theta,
                          DoubleLocalPlannerParam max_brk_x = localPlannerConfig().robot_brk_max_x,
                          DoubleLocalPlannerParam max_brk_y = localPlannerConfig().robot_brk_max_y,
                          DoubleLocalPlannerParam max_brk_theta = localPlannerConfig().robot_brk_max_theta);

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

    DoubleLocalPlannerParam mass;
    DoubleLocalPlannerParam friction;
    DoubleLocalPlannerParam acc_k_p;
    DoubleLocalPlannerParam acc_k_v;
    DoubleLocalPlannerParam acc_scale;
    DoubleLocalPlannerParam max_vel_x;
    DoubleLocalPlannerParam max_vel_y;
    DoubleLocalPlannerParam max_vel_theta;
    DoubleLocalPlannerParam max_acc_x;
    DoubleLocalPlannerParam max_acc_y;
    DoubleLocalPlannerParam max_acc_theta;
    DoubleLocalPlannerParam max_brk_x;
    DoubleLocalPlannerParam max_brk_y;
    DoubleLocalPlannerParam max_brk_theta;
};
}  // namespace luhsoccer::local_planner