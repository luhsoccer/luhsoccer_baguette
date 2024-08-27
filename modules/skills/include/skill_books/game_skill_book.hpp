#pragma once

#include "skills/skill_book.hpp"

namespace luhsoccer::config_provider {
struct ConfigStore;
}

namespace luhsoccer::skills {

enum class GameSkillNames {

    GO_TO_POINT,
    HALT,
    CONTROLLER_GO_TO_POINT,
    GO_TO_POINT_WITH_HEADING,
    MARK_ENEMY_LOS_VARIABLE,
    MARK_GOALIE_LOS,
    GOALIE_DEFEND_ON_CIRCLE,
    GOALIE_LOS_DEFEND,
    GOALIE_LOS_DEFEND_WO_BALL,
    GO_TO_BALL,
    GET_BALL,
    KICK_BALL,
    MARK_ENEMY_TO_BALL,
    GO_TO_PENALTY_LINE,
    WALL_AT_PENALTY_AREA,
    OCTO_SKILL,
    INTERCEPT_BALL,
    KICK_REFLEX,
    STEAL_BALL,
    DRIBBLE,
    PLACE_BALL,
    INTERCEPT_BALL_AT_POINT,
};  // DO NOT MODIFY THIS LINE

class GameSkillBook : public SkillBook<GameSkillNames> {
   public:
    explicit GameSkillBook(const config_provider::ConfigStore& cs);
};
}  // namespace luhsoccer::skills