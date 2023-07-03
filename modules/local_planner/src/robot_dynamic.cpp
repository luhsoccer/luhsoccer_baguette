
#include <utility>

#include "local_planner/robot_dynamic.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::local_planner {

RobotDynamic::RobotDynamic(DoubleLocalPlannerParam mass, DoubleLocalPlannerParam friction,
                           DoubleLocalPlannerParam acc_k_p, DoubleLocalPlannerParam acc_k_v,
                           DoubleLocalPlannerParam acc_scale, DoubleLocalPlannerParam max_vel_x,
                           DoubleLocalPlannerParam max_vel_y, DoubleLocalPlannerParam max_vel_theta,
                           DoubleLocalPlannerParam max_acc_x, DoubleLocalPlannerParam max_acc_y,
                           DoubleLocalPlannerParam max_acc_theta, DoubleLocalPlannerParam max_brk_x,
                           DoubleLocalPlannerParam max_brk_y, DoubleLocalPlannerParam max_brk_theta)
    : mass(std::move(mass)),
      friction(std::move(friction)),
      acc_k_p(std::move(acc_k_p)),
      acc_k_v(std::move(acc_k_v)),
      acc_scale(std::move(acc_scale)),
      max_vel_x(std::move(max_vel_x)),
      max_vel_y(std::move(max_vel_y)),
      max_vel_theta(std::move(max_vel_theta)),
      max_acc_x(std::move(max_acc_x)),
      max_acc_y(std::move(max_acc_y)),
      max_acc_theta(std::move(max_acc_theta)),
      max_brk_x(std::move(max_brk_x)),
      max_brk_y(std::move(max_brk_y)),
      max_brk_theta(std::move(max_brk_theta)) {}

std::pair<Eigen::Matrix<double, SIZE_OF_ROBOT_STATE_VEC, SIZE_OF_ROBOT_STATE_VEC>,
          Eigen::Matrix<double, SIZE_OF_ROBOT_STATE_VEC, SIZE_OF_CMD_VECTOR>>
RobotDynamic::getSystemMatrices(double step_duration) const {
    // NOLINTNEXTLINE(readability-identifier-naming) A has to be uppercase because it is done so in all ref. literature
    Eigen::Matrix<double, SIZE_OF_ROBOT_STATE_VEC, SIZE_OF_ROBOT_STATE_VEC> system_matrix_A;
    // NOLINTNEXTLINE(readability-identifier-naming) A has to be uppercase because it is done so in all ref. literature
    Eigen::Matrix<double, SIZE_OF_ROBOT_STATE_VEC, SIZE_OF_CMD_VECTOR> system_matrix_B;
    double& t = step_duration;
    double m = this->mass;
    double f = this->friction;
    // clang-format off
    system_matrix_A << 1, 0, 0, t, 0, 0,
                      0, 1, 0, 0, t, 0,
                      0, 0, 1, 0, 0, t,
                      0, 0, 0, 1-f*t/m, 0, 0,
                      0, 0, 0, 0, 1-f*t/m, 0,
                      0, 0, 0, 0, 0, 1-f*t/m;
    system_matrix_B << t*t/2, 0, 0,
                      0, t*t/2, 0,
                      0, 0, t*t/2,
                      t, 0, 0,
                      0, t, 0,
                      0, 0, t;

    // clang-format on
    return {system_matrix_A, system_matrix_B};
}

double RobotDynamic::clampAcceleration(double force, double vel, double max_acc, double max_brk) {
    if (max_acc < 0.0 || max_brk < 0.0) throw std::runtime_error("max_acc and max_brk cannot be negative!");
    if (force * vel > 0.0) {
        return std::min(max_acc, std::max(-max_acc, force));
    } else {
        return std::min(max_brk, std::max(-max_brk, force));
    }
}

double RobotDynamic::clampVelocity(double velocity, double max, std::optional<double> min) {
    if (!min) min = -max;
    return std::min(max, std::max(*min, velocity));
}

RobotState RobotDynamic::applyAcceleration(const RobotState& old_state, Eigen::Vector3d acc,
                                           time::Duration duration) const {
    double duration_in_s = duration.asSec();
    auto [A, B] = this->getSystemMatrices(duration_in_s);

    // clamp input force aka. acc
    acc.x() = RobotDynamic::clampAcceleration(acc.x(), old_state[STATE_VX], this->max_acc_x, this->max_brk_x);
    acc.y() = RobotDynamic::clampAcceleration(acc.y(), old_state[STATE_VY], this->max_acc_y, this->max_brk_y);
    acc.z() = RobotDynamic::clampAcceleration(acc.z(), old_state[STATE_VT], this->max_acc_theta, this->max_brk_theta);

    RobotState new_state = A * old_state + B * acc;

    // clamp velocity in local coordinates

    new_state[STATE_VT] = RobotDynamic::clampVelocity(new_state[STATE_VT], this->max_vel_theta);

    Eigen::Vector2d local_vel = Eigen::Rotation2Dd(new_state[STATE_T]).toRotationMatrix().inverse() *
                                Eigen::Vector2d{new_state[STATE_VX], new_state[STATE_VY]};

    local_vel.x() = RobotDynamic::clampVelocity(local_vel.x(), this->max_vel_x);
    local_vel.y() = RobotDynamic::clampVelocity(local_vel.y(), this->max_vel_y);

    Eigen::Vector2d global_vel = Eigen::Rotation2Dd(new_state[STATE_T]).toRotationMatrix() * local_vel;
    new_state[STATE_VX] = global_vel.x();
    new_state[STATE_VY] = global_vel.y();
    return new_state;
}

RobotState RobotDynamic::applyRobotCommandVelocity(RobotState robot_state, const Eigen::Vector3d& velocity,
                                                   time::Duration duration) const {
    // cmd_before = robot_state.tail(3);
    Eigen::Vector3d vel_local = robot_state.tail(3);
    Eigen::Matrix2d rotation_matrix = Eigen::Rotation2Dd(robot_state.z()).toRotationMatrix();
    vel_local.head(2) = rotation_matrix.inverse() * vel_local.head(2);
    Eigen::Vector3d vel_diff = velocity - vel_local;

    Eigen::Vector3d acc = static_cast<double>(this->acc_k_v) * vel_local +
                          static_cast<double>(this->acc_k_p) / duration.asSec() * vel_diff;

    auto limit_acc = [](double& diff, double value_before, double increase_max, double decrease_max) {
        if (diff * value_before > 0.0) {
            diff *= std::min(1.0, increase_max / abs(diff));
        } else {
            diff *= std::min(1.0, decrease_max / abs(diff));
        }
    };
    limit_acc(acc.x(), vel_local.x(), this->max_acc_x * this->acc_scale, this->max_brk_x * this->acc_scale);
    limit_acc(acc.y(), vel_local.y(), this->max_acc_y * this->acc_scale, this->max_brk_y * this->acc_scale);
    limit_acc(acc.z(), vel_local.z(), this->max_acc_theta * this->acc_scale, this->max_acc_theta * this->acc_scale);

    // convert back to global
    acc.head(2) = rotation_matrix * acc.head(2);

    auto [A, B] = this->getSystemMatrices(duration.asSec());
    robot_state = A * robot_state + B * acc;
    // auto limit_velocity = [](double& value, double max) { value *= std::min(1.0, max / abs(value)); };
    // limit_velocity(robot_state[3], config.robot_vel_max_x);
    // limit_velocity(robot_state[4], config.robot_vel_max_y);
    // limit_velocity(robot_state[5], config.robot_vel_max_theta);
    return robot_state;
}

RobotState convertToRobotState(const Eigen::Affine2d& pos, const Eigen::Vector3d& vel) {
    RobotState s;
    s.head(3) << pos.translation().x(), pos.translation().y(), Eigen::Rotation2Dd(pos.rotation()).angle();
    s.tail(3) = vel;

    return s;
}

std::pair<Eigen::Affine2d, Eigen::Vector3d> convertFromRobotState(const RobotState& state) {
    Eigen::Affine2d pos = Eigen::Translation2d(state[0], state[1]) * Eigen::Rotation2Dd(state[2]);
    Eigen::Vector3d vel = state.tail(3);
    return {pos, vel};
}

}  // namespace luhsoccer::local_planner