#pragma once

#include "local_planner/skills/abstract_shape.hpp"
#include "transform/world_model.hpp"

namespace luhsoccer::tests {
void testShape(const local_planner::AbstractShape& shape, const std::vector<std::string> shape_frame,
               const std::vector<Eigen::Vector3d> shape_start, const Eigen::Vector3d ref_start,
               const std::optional<Eigen::Vector2d> expected_vec,
               const std::optional<Eigen::Vector2d> expected_vel = {{0.0, 0.0}},
               const Eigen::Vector3d shape_move = {0.0, 0.0, 0.0}, const Eigen::Vector3d ref_move = {0.0, 0.0, 0.0},
               const time::TimePoint& start_time = time::TimePoint(0), const time::TimePoint& time = time::TimePoint(0),
               const std::string& observe_frame = "global_frame");

void test_res(local_planner::VectorWithVelocityStamped& res, const time::TimePoint& check_time,
              const std::optional<Eigen::Vector2d> expected_vec,
              const std::optional<Eigen::Vector2d> expected_vel = std::nullopt, bool has_value = true);
}  // namespace luhsoccer::tests