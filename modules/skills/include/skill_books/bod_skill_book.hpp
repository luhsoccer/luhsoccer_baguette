#pragma once

#include "skills/skill_book.hpp"

namespace luhsoccer::config_provider {
struct ConfigStore;
}

namespace luhsoccer::skills {

enum class BodSkillNames {
    // GO_TO_POINT,
    // GET_BALL,
    // GO_TO_POINT_ALIGNED,
    // HALT,
    // MARK_ENEMY_TO_BALL,
    // BLOCK_ENEMY_LINE_OF_SIGHT,
    // MOVE_TO_PENALTY_LINE,
    // MARK_ENEMY_TO_GOAL,
    // WALL_AT_PENALTY_AREA,
    // KICK_BALL_THROUGH_TARGET,
    // STEAL_BALL,
    // INTERCEPT_BALL,
    // INTERCEPT_BALL_GOALIE,
    // DEFEND_GOAL_ON_CIRCLE,
    // BLOCK_GOALIE_LO_S,
    // REFLEX_KICK,
    // PREPARE_KICK,
    // DEFEND_GOALLINE,
    // WALL_AT_DISTANCE,
    // BLOCK_ENEMY_LOS_VARIABLE,
    // GO_TO_POINT_WITH_HEADING,
    // MOVE_TO_BALL_TURN_RADIUS,
    // DRIVE_TO_LINE,
    // OKTO_SKILL,
    // DRIVE_IN_BALL,
    // BACKWARDS_DRIBBLING,
    // DRIVE_TO_LINE_SEGMENT,
    // FORWARDS_DRIBBLING,
    // MOVE_CONSTANT,
    // MOVE_CONSTANT2,
    // RECEIVE_BALL_AT_POINT,
    // GOALIE_LOS_DEFEND,
    // GOALIE_LOS_DEFEND_W_O_BALL,
    // INTERCEPT_BALL_REFLEX,
};  // DO NOT MODIFY THIS LINE

class BodSkillBook : public SkillBook<BodSkillNames> {
   public:
    explicit BodSkillBook(const config_provider::ConfigStore& cs);
};
}  // namespace luhsoccer::skills