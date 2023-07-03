#include <gtest/gtest.h>

#include "local_planner/skills/task.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "../world_model_test_helper.hpp"
#include "shape_test_helpers.hpp"

namespace luhsoccer::local_planner {
TEST(circle_shape, closest_point) {
    std::string child_frame_circle_shape = "child_frame_circle_shape";
    luhsoccer::local_planner::CircleShape circle({child_frame_circle_shape}, {2}, {false});

    luhsoccer::tests::testShape(circle, {child_frame_circle_shape}, {{5.0, 3.0, 0.0}}, {1.0, 3.0, 0.0}, {{2.0, 0.0}});
    luhsoccer::tests::testShape(circle, {child_frame_circle_shape}, {{5.0, 3.0, 0.0}}, {5.0, 4.0, 0.0}, {{0.0, 1.0}});
    luhsoccer::tests::testShape(circle, {child_frame_circle_shape}, {{5.0, 3.0, 0.0}}, {8.0, 6.0, 0.0},
                                {{-1.585786, -1.585786}});
    luhsoccer::tests::testShape(circle, {child_frame_circle_shape}, {{5.0, 3.0, 0.0}}, {5.0, 3.0, 0.0}, {{0.0, 0.0}});
};

TEST(circle_shape, velocity) {
    std::string child_frame_circle_shape = "child_frame_circle_shape";
    luhsoccer::local_planner::CircleShape circle({child_frame_circle_shape}, {2}, {false});

    luhsoccer::tests::testShape(circle, {child_frame_circle_shape}, {{5.0, 3.0, 0.0}}, {1.0, 3.0, 0.0}, {},
                                {{0.0, 50.0}}, {0.0, 50.0, 0.0}, {0.0, 0.0, 0.0});
};

TEST(circle_shape, rotation) {
    std::string child_frame_circle_shape = "child_frame_circle_shape";
    luhsoccer::local_planner::CircleShape circle({child_frame_circle_shape}, {2}, {false});
    luhsoccer::tests::testShape(circle, {child_frame_circle_shape}, {{5.0, 3.0, 0.0}}, {1.0, 3.0, 4.0 * L_PI},
                                {{52.0, 0.0}}, {{50.0, 0.0}}, {50.0, 0.0, 0.4 * L_PI}, {0.0, 0.0, 0.0},
                                time::TimePoint(1), time::TimePoint(2));
};

TEST(filled_circle_shape, closest_point) {
    std::string child_frame_circle_shape = "child_frame_circle_shape";
    luhsoccer::local_planner::CircleShape circle({child_frame_circle_shape}, {2}, {true});

    luhsoccer::tests::testShape(circle, {child_frame_circle_shape}, {{5.0, 3.0, 0.0}}, {5.0, 4.0, 0.0}, {{0.0, 0.0}});
    luhsoccer::tests::testShape(circle, {child_frame_circle_shape}, {{5.0, 3.0, 0.0}}, {1.0, 3.0, 0.0}, {{2.0, 0.0}});
    luhsoccer::tests::testShape(circle, {child_frame_circle_shape}, {{5.0, 3.0, 0.0}}, {5.0, 3.0, 0.0}, {{0.0, 0.0}});
};

TEST(filled_circle_shape, velocity) {
    std::string child_frame_circle_shape = "child_frame_circle_shape";
    luhsoccer::local_planner::CircleShape circle({child_frame_circle_shape}, {2}, {true});

    luhsoccer::tests::testShape(circle, {child_frame_circle_shape}, {{5.0, 3.0, 0.0}}, {1.0, 3.0, 0.0}, {},
                                {{0.0, 50.0}}, {0.0, 50.0, 0.0}, {0.0, 0.0, 0.0});
};

TEST(filled_circle_shape, rotation) {
    std::string child_frame_circle_shape = "child_frame_circle_shape";
    luhsoccer::local_planner::CircleShape circle({child_frame_circle_shape}, {2}, {true});
    luhsoccer::tests::testShape(circle, {child_frame_circle_shape}, {{5.0, 3.0, 0.0}}, {1.0, 3.0, 4.0 * L_PI},
                                {{52.0, 0.0}}, {{50.0, 0.0}}, {50.0, 0.0, 0.4 * L_PI}, {0.0, 0.0, 0.0},
                                time::TimePoint(1), time::TimePoint(2));
};
}  // namespace luhsoccer::local_planner
