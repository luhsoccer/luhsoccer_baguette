#pragma once

#include "skills/skill_book.hpp"

namespace luhsoccer::config_provider
{
struct ConfigStore;
}

namespace luhsoccer::skills
{

enum class BodSkillNames
{
    GO_TO_POINT,
    GET_BALL,
    GO_TO_POINT_ALIGNED,
    HALT,
    MARK_ENEMY_TO_BALL,
    BLOCK_ENEMY_LINE_OF_SIGHT,
    MOVE_TO_PENALTY_LINE,
    MARK_ENEMY_TO_GOAL,
    WALL_AT_PENALTY_AREA,
    KICK_BALL_THROUGH_TARGET,
    KICK_BALL_TO_TARGET,
    STEAL_BALL,
    RECEIVE_BALL,
    DEFEND_GOAL_ON_CIRCLE,
    BLOCK_GOALIE_LO_S,
    PASS_BALL_TO_ROBOT,
    REFLEX_KICK,
    PREPARE_KICK,
    DEFEND_GOALLINE,
    WALL_AT_DISTANCE,
    BLOCK_ENEMY_LOS_VARIABLE,
    GO_TO_POINT_WITH_HEADING,
    MOVE_TO_BALL_TURN_RADIUS,
    DRIVE_TO_LINE,
    KICK_BALL_THROUGH_TARGET_PREPOSITION,
    DRIVE_BY_SIDE,
    KICK_BALL_THROUGH_TARGET_ORIGINAL,
    KICK_BALL_THROUGH_TARGET_DIRECT,
    RUN_FREE,
    OKTO_SKILL,
    DRIVE_IN_BALL,
    GO_TO_POINT_ORIGINAL,
    GO_TO_POINT_WITH_HEADING_ORIGINAL,
    GO_TO_POINT_DRIBBLER,
    BACKWARDS_DRIBBLING,
    DRIVE_TO_LINE_SEGMENT,
    GO_TO_POINT_INTERCEPT,
    FORWARDS_DRIBBLING,
};  // DO NOT MODIFY THIS LINE

class BodSkillBook : public SkillBook<BodSkillNames>
{
   public:
    explicit BodSkillBook(const config_provider::ConfigStore& cs);
};
}  // namespace luhsoccer::skills