#include "skill_books/game_skill_book/mark_enemy_los_variable.hpp"
#include "config/skills_config.hpp"
#include "robot_control/components/component_data.hpp"
#include "robot_control/components/features/target_feature.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/shapes/circle_shape.hpp"
#include "robot_control/components/shapes/line_shape.hpp"
#include "robot_control/components/steps/dribbler_step.hpp"
#include "robot_control/components/steps/drive_step.hpp"
// include components here

namespace luhsoccer::skills {

MarkEnemyLosVariableBuild::MarkEnemyLosVariableBuild()
    : SkillBuilder("MarkEnemyLosVariable",  //
                   {"Enemy ball carrier"},  //
                   {},                      //
                   {"Defend radius"},       //
                   {},                      //
                   {},                      //
                   {}){};

void MarkEnemyLosVariableBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    addStep(DribblerStep(robot_interface::DribblerMode::OFF));
    DriveStep drive_to_line_of_sight;

    drive_to_line_of_sight.setReachCondition(DriveStep::ReachCondition::NEVER);
    drive_to_line_of_sight.setRotationControl(HeadingRotationControl("ball"));

    ComponentPosition line_end(
        CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> transform::Position {
            auto ball = transform::Position("ball").getCurrentPosition(comp_data.wm);
            auto enemy =
                transform::Position(comp_data.td.related_robots[0].getFrame()).getCurrentPosition(comp_data.wm);

            if (!ball.has_value() || !enemy.has_value()) return "";
            Eigen::Vector2d end_pos = ball->translation() + (ball->translation() - enemy->translation()).normalized() *
                                                                comp_data.td.required_doubles[0];
            // check if end pos out of field

            auto field_data = comp_data.wm->getFieldData();

            // check if out of side line
            if (abs(end_pos.y()) > field_data.size.y() / 2) {
                double offset = std::sqrt(std::pow(comp_data.td.required_doubles[0], 2) -
                                          std::pow(field_data.size.y() / 2 - std::abs(ball->translation().y()), 2));
                // move to opposite side of ball than enemy
                if (ball->translation().x() < enemy->translation().x()) {
                    offset *= -1;
                }
                end_pos.x() = ball->translation().x() + offset;
                end_pos.y() = std::min(std::max(end_pos.y(), -field_data.size.y() / 2), field_data.size.y() / 2);
            }

            if (abs(end_pos.x()) > field_data.size.x() / 2) {
                double offset = std::sqrt(std::pow(comp_data.td.required_doubles[0], 2) -
                                          std::pow(field_data.size.x() / 2 - std::abs(ball->translation().x()), 2));
                // move to opposite side of ball than enemy if in other half
                // always move towards our goal if in our half
                if ((end_pos.x() > 0.0 && ball->translation().y() < enemy->translation().y()) ||
                    (end_pos.x() < 0.0 && ball->translation().y() > 0.0)) {
                    offset *= -1;
                }
                end_pos.y() = ball->translation().y() + offset;
                end_pos.x() = std::min(std::max(end_pos.x(), -field_data.size.x() / 2), field_data.size.x() / 2);
            }
            // extend end pos to max intercept distance
            constexpr double MAX_INTERCEPT_DISTANCE = 3.0;
            end_pos += (end_pos - ball->translation()).normalized() *
                       std::max(MAX_INTERCEPT_DISTANCE - comp_data.td.required_doubles[0], 0.0);

            return {"", end_pos.x(), end_pos.y()};
        });

    constexpr double SMALL_VALUE = 0.1;
    DoubleComponentParam cutoff_start(CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> double {
        return comp_data.td.required_doubles[0] - SMALL_VALUE;
    });
    drive_to_line_of_sight.addFeature(TargetFeature(LineShape("ball", line_end, cutoff_start, 0.0)));

    TargetFeature circle_target(CircleShape("ball", {TD, 0}, false));
    circle_target.setWeight(cs.skills_config.marking_distance_weight);
    drive_to_line_of_sight.addFeature(std::move(circle_target));
    drive_to_line_of_sight.activateConstraint(DriveStepConstraintNames::ENEMY_KICKOFF, true);

    addStep(std::move(drive_to_line_of_sight));

    // end of skill
}
}  // namespace luhsoccer::skills