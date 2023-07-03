#define MOVE_TO_BALL_TURN_RADIUS                                                                    \
    addStep(DribblerStep(robot_interface::DribblerMode::OFF));                                      \
    DriveStep go_to_get_ball;                                                                       \
    go_to_get_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);                    \
    go_to_get_ball.setAvoidOtherRobots(true);                                                       \
    go_to_get_ball.setRotationControl(HeadingRotationControl("ball", false));                       \
    go_to_get_ball.addFeature(                                                                      \
        TargetFeature(CircleShape("ball", cs.skills_config.move_to_ball_pre_radius, false), 0.05)); \
    addStep(std::move(go_to_get_ball));
