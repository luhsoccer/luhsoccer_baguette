#include <gtest/gtest.h>

#include "local_planner/skills/task.hpp"
#include "local_planner_components/shapes/arc_shape.hpp"
#include "../world_model_test_helper.hpp"
#include "shape_test_helpers.hpp"

namespace luhsoccer::local_planner {
TEST(arc_shape, closest_point) {
    std::string child_frame_arc_shape = "child_frame_arc_shape";
    ArcShape arc_90({{child_frame_arc_shape, 0.0, 0.0, L_PI / 4}}, {2}, {L_PI / 2});
    ArcShape arc_180({{child_frame_arc_shape, 0.0, 0.0, L_PI / 2}}, {2}, {L_PI});
    ArcShape arc_270({{child_frame_arc_shape, 0.0, 0.0, L_PI / 0.75}}, {2}, {1.5 * L_PI});

    std::vector<ArcShape> shapes = {arc_90, arc_180, arc_270};
    std::vector<Eigen::Vector3d> ref_starts = {{0.0, 2.0, 0.0}, {1.0, 2.0, 0.0}, {2.0, 2.0, 0.0}, {3.0, 2.0, 0.0},
                                               {4.0, 2.0, 0.0}, {5.0, 2.0, 0.0}, {0.0, 5.0, 0.0}};
    std::vector<std::optional<Eigen::Vector2d>> expected_arc_90 = {
        {{2.0, 2.0}}, {{1.0, 2.0}}, {}, {{1.0, 0.0}}, {{0.0, 0.0}}, {{-1.0, 0.0}}, {{2.0, -1.0}}};
    std::vector<std::optional<Eigen::Vector2d>> expected_arc_180 = {{{0.0, 0.0}}, {{-1.0, 0.0}}, {}, {{1.0, 0.0}},
                                                                    {{0.0, 0.0}}, {{-1.0, 0.0}}, {}};
    std::vector<std::optional<Eigen::Vector2d>> expected_arc_270 = {{{0.0, 0.0}}, {{-1.0, 0.0}}, {}, {{1.0, 0.0}},
                                                                    {{0.0, 0.0}}, {{-1.0, 0.0}}, {}};
    std::vector<std::vector<std::optional<Eigen::Vector2d>>> expected = {expected_arc_90, expected_arc_180,
                                                                         expected_arc_270};

    // Iterate through all shapes
    for (int s = 0; s < shapes.size(); s++) {
        for (int i = 0; i < ref_starts.size(); i++) {
            luhsoccer::tests::testShape(shapes[s], {child_frame_arc_shape}, {{2.0, 2.0, 0.0}}, ref_starts[i],
                                        expected[s][i]);
        }
    }
};

TEST(arc_shape, velocity) {
    std::string child_frame_arc_shape = "child_frame_arc_shape";
    ArcShape shape({child_frame_arc_shape}, 1.0, L_PI);

    luhsoccer::tests::testShape(shape, {child_frame_arc_shape}, {{0.0, 0.0, 0.0}}, {3.0, 1.0, 0.0}, {}, {{1.0, 0.0}},
                                {0.0, 0.0, 0.0}, {-1.0, 0.0, 0.0});
    luhsoccer::tests::testShape(shape, {child_frame_arc_shape}, {{0.0, 0.0, 0.0}}, {-3.0, 1.0, 0.0}, {}, {{-2.0, 0.0}},
                                {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0});
}

}  // namespace luhsoccer::local_planner
