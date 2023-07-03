#include <gtest/gtest.h>

#include "local_planner/skills/task.hpp"
#include "local_planner_components/shapes/compose_shape.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "../world_model_test_helper.hpp"
#include "shape_test_helpers.hpp"

namespace luhsoccer::local_planner {
TEST(compose_shape, closest_point) {
    std::shared_ptr<transform::WorldModel> wm = std::make_shared<transform::WorldModel>("global_frame");
    local_planner::TaskData td(wm->getPossibleRobots().at(0));
    time::TimePoint start_time = time::TimePoint(0.5);
    time::TimePoint check_time = time::TimePoint(1.0);

    // Setup Circle
    std::string child_frame_circle_shape = "child_frame_circle_shape";
    CircleShape shape({child_frame_circle_shape}, {2.0}, {false});
    luhsoccer::tests::pushFrameToWorldmodel(wm, child_frame_circle_shape, start_time, 2.0, 2.0, 0.0);

    // Setup Line
    std::string child_frame_line_shape_start = "child_frame_line_shape_start";
    luhsoccer::tests::pushFrameToWorldmodel(wm, child_frame_line_shape_start, start_time, 4.0, 2.0, 0.0);
    std::string child_frame_line_shape_end = "child_frame_line_shape_end";
    luhsoccer::tests::pushFrameToWorldmodel(wm, child_frame_line_shape_end, start_time, 8.0, 2.0, 0.0);
    LineShape line({child_frame_line_shape_start}, {child_frame_line_shape_end});

    // Setup reference frame
    luhsoccer::tests::pushFrameToWorldmodel(wm, "ref", start_time, 3.0, 2.0, 0.0);

    // Setup ComposeShape
    ComposeShape cs({std::make_shared<CircleShape>(shape), std::make_shared<LineShape>(line)});

    VectorWithVelocityStamped res = cs.getTransformToClosestPoint(wm, td, {"ref"}, check_time);
    tests::test_res(res, check_time, {{1.0, 0.0}}, {{0.0, 0.0}});

    // Test for no shapes in compose shape
    ComposeShape cs2;
    res = cs2.getTransformToClosestPoint(wm, td, {"ref"}, check_time);
    tests::test_res(res, check_time, {}, {}, false);

    // Test for one shape in compose shape
    ComposeShape cs3({std::make_shared<CircleShape>(shape)});
    res = cs3.getTransformToClosestPoint(wm, td, {"ref"}, check_time);
    tests::test_res(res, check_time, {{1.0, 0.0}}, {{0.0, 0.0}});
};

TEST(compose_shape, velocity) {
    std::shared_ptr<transform::WorldModel> wm = std::make_shared<transform::WorldModel>("global_frame");
    local_planner::TaskData td(wm->getPossibleRobots().at(0));
    time::TimePoint start_time = time::TimePoint(0.5);
    time::TimePoint check_time = time::TimePoint(1.0);

    // Setup Circle
    std::string child_frame_circle_shape = "child_frame_circle_shape";
    CircleShape shape({child_frame_circle_shape}, {2.0}, {false});
    luhsoccer::tests::pushFrameToWorldmodel(wm, child_frame_circle_shape, start_time, 2.0, 2.0, 0.0);

    // Setup Line
    std::string child_frame_line_shape_start = "child_frame_line_shape_start";
    luhsoccer::tests::pushFrameToWorldmodel(wm, child_frame_line_shape_start, start_time, 4.0, 2.0, 0.0);
    std::string child_frame_line_shape_end = "child_frame_line_shape_end";
    luhsoccer::tests::pushFrameToWorldmodel(wm, child_frame_line_shape_end, start_time, 8.0, 2.0, 0.0);
    LineShape line({child_frame_line_shape_start}, {child_frame_line_shape_end});

    // Setup reference frame
    luhsoccer::tests::pushFrameToWorldmodel(wm, "ref", start_time, 3.0, 2.0, 0.0, -1.0, -1.0, 0.0);

    // Setup ComposeShape
    ComposeShape cs({std::make_shared<CircleShape>(shape), std::make_shared<LineShape>(line)});

    VectorWithVelocityStamped res = cs.getTransformToClosestPoint(wm, td, {"ref"}, check_time);
    tests::test_res(res, check_time, {}, {{1.0, 1.0}});
}
}  // namespace luhsoccer::local_planner
