#include "shape_test_helpers.hpp"
#include <gtest/gtest.h>
#include "local_planner/skills/task.hpp"
#include "../world_model_test_helper.hpp"

namespace luhsoccer::tests {
/**
 * @brief Tests the VectorWithVelocityStamped result of getTransformToClosestPoint
 *
 * @param res
 * @param check_time
 * @param expected_vec
 * @param expected_vel
 * @param has_value Should the result vector have a value?
 */
void test_res(local_planner::VectorWithVelocityStamped& res, const time::TimePoint& check_time,
              const std::optional<Eigen::Vector2d> expected_vec, const std::optional<Eigen::Vector2d> expected_vel,
              bool has_value) {
    EXPECT_EQ(res.stamp, check_time);
    EXPECT_EQ(res.vec.has_value(), has_value);
    if (expected_vec.has_value()) {
        EXPECT_NEAR(res.vec->x(), expected_vec->x(), 1e-6);
        EXPECT_NEAR(res.vec->y(), expected_vec->y(), 1e-6);
    }
    if (expected_vel.has_value()) {
        EXPECT_NEAR(res.velocity->x(), expected_vel->x(), 1e-6);
        EXPECT_NEAR(res.velocity->y(), expected_vel->y(), 1e-6);
    }
}

/**
 * @brief Tests the results of the closestPoint method of a shape
 *
 * @param shape
 * @param shape_frame The frame of the shape
 * @param shape_start The starting position and rotation of the shape
 * @param ref_start The starting position and rotation of the reference frame
 * @param expected_vec The expected vector from reference to shape, use {} to skip check on this
 * @param expected_vel The expected velocity, use {} to skip check on this
 * @param shape_move The movement of the shape, default is no movement
 * @param ref_move The movement of the reference frame, default is no movement
 * @param time The timepoint to test, default is 0
 * @param observe_frame The frame to observe the shape in, default global_frame
 */
void testShape(const local_planner::AbstractShape& shape, const std::vector<std::string> shape_frame,
               const std::vector<Eigen::Vector3d> shape_start, const Eigen::Vector3d ref_start,
               const std::optional<Eigen::Vector2d> expected_vec, const std::optional<Eigen::Vector2d> expected_vel,
               const Eigen::Vector3d shape_move, const Eigen::Vector3d ref_move, const time::TimePoint& start_time,
               const time::TimePoint& time, const std::string& observe_frame) {
    const std::string ref_frame = "ref_frame_for_testing";

    std::shared_ptr<transform::WorldModel> wm = std::make_shared<transform::WorldModel>(observe_frame);
    local_planner::TaskData td(wm->getPossibleRobots().at(0));

    for (int i = 0; i < shape_frame.size(); i++) {
        luhsoccer::tests::pushFrameToWorldmodel(wm, shape_frame[i], start_time, shape_start[i].x(), shape_start[i].y(),
                                                shape_start[i].z(), shape_move.x(), shape_move.y(), shape_move.z());
    }
    luhsoccer::tests::pushFrameToWorldmodel(wm, ref_frame, start_time, ref_start.x(), ref_start.y(), ref_start.z(),
                                            ref_move.x(), ref_move.y(), ref_move.z());

    local_planner::VectorWithVelocityStamped res =
        shape.getTransformToClosestPoint(wm, td, {ref_frame}, time, {observe_frame});

    test_res(res, time, expected_vec, expected_vel);
}
}  // namespace luhsoccer::tests