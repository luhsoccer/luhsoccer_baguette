#include <gtest/gtest.h>

#include "local_planner/skills/task.hpp"
#include "local_planner_components/shapes/rectangle_shape.hpp"
#include "../world_model_test_helper.hpp"
#include "shape_test_helpers.hpp"
#include "config_provider/config_store_main.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::local_planner {
TEST(rectangle_shape, closest_point) {
    std::string child_frame_rect_shape = "child_frame_rectangle_shape";
    RectangleShape shape({child_frame_rect_shape}, {2}, {2}, {false});

    luhsoccer::tests::testShape(shape, {child_frame_rect_shape}, {{1.0, 1.0, 0.0}}, {1.0, 3.0, 0.0}, {{0.0, -1.0}});
    luhsoccer::tests::testShape(shape, {child_frame_rect_shape}, {{1.0, 1.0, 0.0}}, {1.0, 1.0, 0.0}, {});
    luhsoccer::tests::testShape(shape, {child_frame_rect_shape}, {{1.0, 1.0, 0.0}}, {1.6, 1.0, 0.0}, {{0.4, 0.0}});

    RectangleShape shape2({child_frame_rect_shape}, {2}, {2}, {true});

    luhsoccer::tests::testShape(shape2, {child_frame_rect_shape}, {{1.0, 1.0, 0.0}}, {1.0, 3.0, 0.0}, {{0.0, -1.0}});
    luhsoccer::tests::testShape(shape2, {child_frame_rect_shape}, {{1.0, 1.0, 0.0}}, {1.0, 1.0, 0.0}, {{0.0, 0.0}});
    luhsoccer::tests::testShape(shape2, {child_frame_rect_shape}, {{1.0, 1.0, 0.0}}, {1.6, 1.0, 0.0}, {{0.0, 0.0}});
};

TEST(rectangle_shape, velocity) {
    std::string child_frame_rect_shape = "child_frame_rectangle_shape";
    RectangleShape shape({child_frame_rect_shape}, {2}, {2}, {false});

    luhsoccer::tests::testShape(shape, {child_frame_rect_shape}, {{0.0, 0.0, 0.0}}, {3.0, 1.0, 0.0}, {}, {{1.0, 0.0}},
                                {0.0, 0.0, 0.0}, {-1.0, 0.0, 0.0});
    luhsoccer::tests::testShape(shape, {child_frame_rect_shape}, {{0.0, 0.0, 0.0}}, {-3.0, 1.0, 0.0}, {}, {{-2.0, 0.0}},
                                {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0});
}

TEST(rectangle_shape, rotation) {
    std::string child_frame_rect_shape = "child_frame_rectangle_shape";
    RectangleShape shape({child_frame_rect_shape}, {2}, {2}, {false});
    luhsoccer::tests::testShape(shape, {child_frame_rect_shape}, {{1.0, 1.0, 0.25 * L_PI}}, {1.0, 3.0, 0.0},
                                {{0.0, -(2 - sqrt(2.0))}});

    RectangleShape shape2({child_frame_rect_shape}, {2}, {2}, {true});
    luhsoccer::tests::testShape(shape2, {child_frame_rect_shape}, {{1.0, 1.0, 0.25 * L_PI}}, {1.0, 2.245, 0.0},
                                {{0.0, 0.0}});
}
}  // namespace luhsoccer::local_planner
