#include "skill_books/game_skill_book/intercept_ball.hpp"
#include "config/skills_config.hpp"
#include "game_skill_book/ball_steps.hpp"
#include "robot_control/components/component_data.hpp"
#include "robot_control/components/features/target_feature.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/shapes/circle_shape.hpp"
#include "robot_control/components/shapes/line_shape.hpp"
#include "robot_control/components/steps/dribbler_step.hpp"
#include "robot_control/components/steps/drive_step.hpp"
// include components here

namespace luhsoccer::skills {

InterceptBallBuild::InterceptBallBuild()
    : SkillBuilder("InterceptBall",  //
                   {},               //
                   {},               //
                   {},               //
                   {},               //
                   {},               //
                   {}){};

void InterceptBallBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    addStep(DribblerStep(robot_interface::DribblerMode::LOW));

    addStepPtr(getWaitForShotStep(cs));

    addStepPtr(getPrepositionForInterceptStep(cs, {"ball"}));
    addStep(DribblerStep(robot_interface::DribblerMode::HIGH));

    BoolComponentParam cancel_cond(CALLBACK, [&cs](const ComponentData& comp_data, const ComponentUid&) -> bool {
        auto ball_info = comp_data.wm->getBallInfo();
        auto robot_pos =
            transform::Position(comp_data.td.robot.getFrame()).getCurrentPosition(comp_data.wm, "", comp_data.time);

        // cancel if ball is in dribbler
        if (ball_info.has_value() && ball_info->isInRobot(comp_data.td.robot)) return true;

        if (robot_pos.has_value()) {
            auto field_size = comp_data.wm->getFieldData().size;

            // cancel if robot is outside of the field + margin
            if (std::abs(robot_pos->translation().x()) > field_size.x() / 2 - cs.skills_config.intercept_ball_margin ||
                std::abs(robot_pos->translation().y()) > field_size.y() / 2 - cs.skills_config.intercept_ball_margin) {
                return true;
            }
        }

        return false;
    });

    DriveStep get_ball;
    get_ball.setRotationControl(HeadingRotationControl("ball"));
    get_ball.setReachCondition(DriveStep::ReachCondition::NEVER);
    TargetFeature line_target2(LineShape(getFutureBallPosition(cs), "ball", -LINE_CUTOFF_LENGTH, 0.0));
    line_target2.setIgnoreVelocity(true);
    line_target2.setVelocityZeroForReach(false);
    line_target2.setTranslationalTolerance(LINE_TRANS_TOLERANCE);

    auto ball_moving = getBallMoving(cs);
    line_target2.setWeight({CALLBACK, [ball_moving](const ComponentData& comp_data, const ComponentUid&) {
                                return ball_moving.val(comp_data) ? 1.0 : 0.0;
                            }});

    get_ball.addFeature(std::move(line_target2));

    BallTargetFeature ball_target(CircleShape("ball", cs.skills_config.ball_get_ball_radius, false));
    ball_target.setKp(cs.skills_config.ball_get_ball_k_p_scaling);
    get_ball.addFeature(std::move(ball_target));
    get_ball.setCancelCondition(cancel_cond);

    addStep(std::move(get_ball));
    // end of skill
}
}  // namespace luhsoccer::skills