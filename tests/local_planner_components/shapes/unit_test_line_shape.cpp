#include <gtest/gtest.h>

#include "local_planner/skills/task.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "../world_model_test_helper.hpp"
#include "shape_test_helpers.hpp"

namespace luhsoccer::local_planner {
TEST(line_shape, closest_point) {
    std::string child_frame_line_shape_start = "child_frame_line_shape_start";
    std::string child_frame_line_shape_end = "child_frame_line_shape_end";
    LineShape line1({child_frame_line_shape_start}, {child_frame_line_shape_end});

    luhsoccer::tests::testShape(line1, {child_frame_line_shape_start, child_frame_line_shape_end},
                                {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}, {1.86, 1.0, 0.0}, {{0.0, -1.0}});
    luhsoccer::tests::testShape(line1, {child_frame_line_shape_start, child_frame_line_shape_end},
                                {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}, {1.0, 0.0, 0.0}, {{0.0, 0.0}});

    std::string child_frame_line_shape2_start = "child_frame_line_shape2_start";
    std::string child_frame_line_shape2_end = "child_frame_line_shape2_end";
    LineShape line2({child_frame_line_shape2_start}, {child_frame_line_shape2_end});
    luhsoccer::tests::testShape(line1, {child_frame_line_shape_start, child_frame_line_shape_end},
                                {{6.0, 5.0, 0.0}, {3.0, 2.0, 0.0}}, {2.0, 1.0, 0.0}, {{1.0, 1.0}});
    luhsoccer::tests::testShape(line1, {child_frame_line_shape_start, child_frame_line_shape_end},
                                {{6.0, 5.0, 0.0}, {3.0, 2.0, 0.0}}, {5.0, 2.0, 0.0}, {{-1.0, 1.0}});
};

TEST(line_shape, cutoff_parameter) {
    std::string child_frame_line_shape_start = "child_frame_line_shape_start";
    std::string child_frame_line_shape_end = "child_frame_line_shape_end";
    LineShape line1({child_frame_line_shape_start}, {child_frame_line_shape_end}, {0.0}, {-1.0});

    luhsoccer::tests::testShape(line1, {child_frame_line_shape_start, child_frame_line_shape_end},
                                {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}, {3.0, 1.0, 0.0}, {{0.0, -1.0}});
    luhsoccer::tests::testShape(line1, {child_frame_line_shape_start, child_frame_line_shape_end},
                                {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}, {5.0, 2.0, 0.0}, {{-2.0, -2.0}});

    std::string child_frame_line_shape2_start = "child_frame_line_shape2_start";
    std::string child_frame_line_shape2_end = "child_frame_line_shape2_end";
    LineShape line2({child_frame_line_shape_start}, {child_frame_line_shape_end}, {0.0}, {1.0});
    luhsoccer::tests::testShape(line2, {child_frame_line_shape_start, child_frame_line_shape_end},
                                {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}, {3.0, 1.0, 0.0}, {{-2.0, -1.0}});
    luhsoccer::tests::testShape(line2, {child_frame_line_shape_start, child_frame_line_shape_end},
                                {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}, {5.0, 2.0, 0.0}, {{-4.0, -2.0}});
};

TEST(line_shape, velocity) {
    std::string child_frame_line_shape_start = "child_frame_line_shape_start";
    std::string child_frame_line_shape_end = "child_frame_line_shape_end";
    LineShape line1({child_frame_line_shape_start}, {child_frame_line_shape_end});

    luhsoccer::tests::testShape(line1, {child_frame_line_shape_start, child_frame_line_shape_end},
                                {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}, {1.0, 1.0, 0.0}, {}, {{0.0, 100.0}},
                                {0.0, 0.0, 0.0}, {0.0, -100.0, 0.0});
};

TEST(line_shape, velocity_rotation) {
    std::string child_frame_line_shape_start = "child_frame_line_shape_start";
    std::string child_frame_line_shape_end = "child_frame_line_shape_end";
    LineShape line1({child_frame_line_shape_start}, {child_frame_line_shape_end});

    luhsoccer::tests::testShape(line1, {child_frame_line_shape_start, child_frame_line_shape_end},
                                {{0.0, 0.0, L_PI}, {2.0, 0.0, 0.0}}, {1.0, 1.0, 0.0}, {{0.0, 99.0}}, {{0.0, 100.0}},
                                {0.0, 0.0, 0.4 * L_PI}, {0.0, -100.0, 3 * L_PI}, time::TimePoint(1),
                                time::TimePoint(2));
};
}  // namespace luhsoccer::local_planner
