#include <gtest/gtest.h>

#include "local_planner/skills/task.hpp"
#include "local_planner_components/shapes/point_shape.hpp"
#include "../world_model_test_helper.hpp"
#include "shape_test_helpers.hpp"

namespace luhsoccer::local_planner {
TEST(point_shape, closest_point) {
    std::string child_frame_point_shape = "child_frame_point_shape";
    PointShape shape({child_frame_point_shape});

    luhsoccer::tests::testShape(shape, {child_frame_point_shape}, {{3.0, 5.0, 0.0}}, {3.0, 1.0, 0.0}, {{0.0, 4.0}});
    luhsoccer::tests::testShape(shape, {child_frame_point_shape}, {{3.0, 5.0, 0.0}}, {4.0, 8.0, 0.0}, {{-1.0, -3.0}});
    luhsoccer::tests::testShape(shape, {child_frame_point_shape}, {{3.0, 5.0, 0.0}}, {3.0, 5.0, 0.0}, {{0.0, 0.0}});
};

TEST(point_shape, velocity) {
    std::string child_frame_point_shape = "child_frame_point_shape";
    PointShape shape({child_frame_point_shape});

    luhsoccer::tests::testShape(shape, {child_frame_point_shape}, {{0.0, 0.0, 0.0}}, {3.0, 1.0, 0.0}, {}, {{1.0, 0.0}},
                                {0.0, 0.0, 0.0}, {-1.0, 0.0, 0.0});
    luhsoccer::tests::testShape(shape, {child_frame_point_shape}, {{0.0, 0.0, 0.0}}, {-3.0, 1.0, 0.0}, {},
                                {{-2.0, 0.0}}, {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0});
}

TEST(point_shape, rotation) {
    std::string child_frame_point_shape = "child_frame_point_shape";
    PointShape shape({child_frame_point_shape});

    luhsoccer::tests::testShape(shape, {child_frame_point_shape}, {{0.0, 0.0, 0.0}}, {3.0, 1.0, 0.0}, {{-2.0, -1.0}},
                                {{1.0, 0.0}}, {0.0, 0.0, 5 * L_PI}, {-1.0, 0.0, 0.3 * L_PI}, time::TimePoint(5),
                                time::TimePoint(6));
    luhsoccer::tests::testShape(shape, {child_frame_point_shape}, {{0.0, 0.0, L_PI}}, {-3.0, 1.0, 0.0}, {{1.0, -1.0}},
                                {{-2.0, 0.0}}, {0.0, 0.0, 0.0}, {2.0, 0.0, 4.5 * L_PI}, time::TimePoint(5),
                                time::TimePoint(6));
}
}  // namespace luhsoccer::local_planner
