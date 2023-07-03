#include "defense_area.hpp"

#include "local_planner_components/features/anti_target_feature.hpp"
#include "local_planner_components/features/robot_cf_obstacle.hpp"
#include "local_planner_components/shapes/point_shape.hpp"
#include "local_planner_components/shapes/rectangle_shape.hpp"
#include "transform/transform.hpp"

namespace luhsoccer::local_planner {

DefenseAreaConstraint::DefenseAreaConstraint()
    : DriveStepConstraint(),
      w1(CALLBACK,
         [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
             auto pos_corner_right =
                 wm->getTransform(td.robot.getFrame(), transform::field::DEFENSE_AREA_CORNER_ENEMY_RIGHT);
             auto pos_intersection_left =
                 wm->getTransform(td.robot.getFrame(), transform::field::DEFENSE_AREA_INTERSECTION_ENEMY_LEFT);

             if (!pos_corner_right.has_value() || !pos_intersection_left.has_value()) return 0.0;

             if (pos_corner_right->transform.translation().x() > -localPlannerConfig().defense_area_margin &&
                 pos_corner_right->transform.translation().y() > -localPlannerConfig().defense_area_margin &&
                 pos_intersection_left->transform.translation().x() < localPlannerConfig().defense_area_margin &&
                 pos_intersection_left->transform.translation().y() < localPlannerConfig().defense_area_margin) {
                 return 0;
             } else {
                 return 1.0;
             }
         }),
      w2(CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
          auto pos_intersection_right =
              wm->getTransform(td.robot.getFrame(), transform::field::DEFENSE_AREA_INTERSECTION_ALLY_RIGHT);
          auto pos_corner_left = wm->getTransform(td.robot.getFrame(), transform::field::DEFENSE_AREA_CORNER_ALLY_LEFT);

          if (!pos_intersection_right.has_value() || !pos_corner_left.has_value()) return 0.0;

          if (pos_intersection_right->transform.translation().x() > -localPlannerConfig().defense_area_margin &&
              pos_intersection_right->transform.translation().y() > -localPlannerConfig().defense_area_margin &&
              pos_corner_left->transform.translation().x() < localPlannerConfig().defense_area_margin &&
              pos_corner_left->transform.translation().y() < localPlannerConfig().defense_area_margin) {
              return 0;
          } else {
              return 1.0;
          }
      }) {
    this->initFeatures();
};

bool DefenseAreaConstraint::conditionMeetImpl(const std::shared_ptr<const transform::WorldModel>& wm,
                                              const TaskData& td) const {
    auto goalie = wm->getGoalieID();
    return !goalie.has_value() || goalie.value() != td.robot;
}

[[nodiscard]] std::vector<std::shared_ptr<const AbstractTargetFeature>>
DefenseAreaConstraint::getAdditionalTargetFeaturesImpl() const {
    DoubleComponentParam w1 = this->w1;
    DoubleComponentParam w1_anti(
        CALLBACK, [w1](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            if (w1.val(wm, td) > 0.5) {
                return 0.0;
            } else {
                return 10000.0;
            }
        });

    DoubleComponentParam w2 = this->w2;
    DoubleComponentParam w2_anti(
        CALLBACK, [w2](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            if (w2.val(wm, td) > 0.5) {
                return 0.0;
            } else {
                return 1000.0;
            }
        });

    DoubleComponentParam k1_anti(
        CALLBACK, [w1](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            if (w1.val(wm, td) > 0.5) {
                return 0.0;
            } else {
                return 2.0;
            }
        });

    DoubleComponentParam k2_anti(
        CALLBACK, [w2](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            if (w2.val(wm, td) > 0.5) {
                return 0.0;
            } else {
                return 2.0;
            }
        });
    return {std::make_shared<AntiTargetFeature>(PointShape({{transform::field::GOAL_ENEMY_CENTER, FIELD_RUNOFF_WIDTH}}),
                                                2.5, w1_anti, std::nullopt, std::nullopt, k1_anti),
            std::make_shared<AntiTargetFeature>(PointShape({{transform::field::GOAL_ALLY_CENTER, -FIELD_RUNOFF_WIDTH}}),
                                                2.5, w2_anti, std::nullopt, std::nullopt, k2_anti)};
}

[[nodiscard]] std::vector<std::shared_ptr<const AbstractCFObstacle>>
DefenseAreaConstraint::getAdditionalObstacleFeaturesImpl() const {
    RectangleShape r1({{transform::field::GOAL_ENEMY_CENTER, FIELD_RUNOFF_WIDTH}},
                      {CALLBACK,
                       [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
                           return wm->getFieldData().penalty_area_width + localPlannerConfig().defense_area_margin * 2;
                       }},
                      {CALLBACK,
                       [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
                           return 2 * (wm->getFieldData().penalty_area_depth + FIELD_RUNOFF_WIDTH +
                                       localPlannerConfig().defense_area_margin);
                       }},
                      false);

    RectangleShape r2({{transform::field::GOAL_ALLY_CENTER, -FIELD_RUNOFF_WIDTH}},
                      {CALLBACK,
                       [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
                           return wm->getFieldData().penalty_area_width + localPlannerConfig().defense_area_margin * 2;
                       }},
                      {CALLBACK,
                       [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
                           return 2 * (wm->getFieldData().penalty_area_depth + FIELD_RUNOFF_WIDTH +
                                       localPlannerConfig().defense_area_margin);
                       }},
                      false);

    return {std::make_shared<RobotCFObstacle>(std::move(r1), this->w1, std::nullopt, std::nullopt,
                                              localPlannerConfig().defense_area_collision_distance),
            std::make_shared<RobotCFObstacle>(std::move(r2), this->w2, std::nullopt, std::nullopt,
                                              localPlannerConfig().defense_area_collision_distance)};
}

}  // namespace luhsoccer::local_planner