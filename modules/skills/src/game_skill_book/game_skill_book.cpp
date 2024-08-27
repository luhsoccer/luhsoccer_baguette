#include "skill_books/game_skill_book.hpp"

// include skill file here

#include "skill_books/game_skill_book/go_to_point.hpp"
#include "skill_books/game_skill_book/halt.hpp"
#include "skill_books/game_skill_book/go_to_point_with_heading.hpp"
#include "skill_books/game_skill_book/controller_go_to_point.hpp"
#include "skill_books/game_skill_book/mark_enemy_los_variable.hpp"
#include "skill_books/game_skill_book/mark_goalie_los.hpp"
#include "skill_books/game_skill_book/goalie_defend_on_circle.hpp"
#include "skill_books/game_skill_book/goalie_los_defend.hpp"
#include "skill_books/game_skill_book/goalie_los_defend_wo_ball.hpp"
#include "skill_books/game_skill_book/go_to_ball.hpp"
#include "skill_books/game_skill_book/get_ball.hpp"
#include "skill_books/game_skill_book/kick_ball.hpp"
#include "skill_books/game_skill_book/mark_enemy_to_ball.hpp"
#include "skill_books/game_skill_book/go_to_penalty_line.hpp"
#include "skill_books/game_skill_book/wall_at_penalty_area.hpp"
#include "skill_books/game_skill_book/octo_skill.hpp"
#include "skill_books/game_skill_book/intercept_ball.hpp"
#include "skill_books/game_skill_book/kick_reflex.hpp"
#include "skill_books/game_skill_book/steal_ball.hpp"
#include "skill_books/game_skill_book/dribble.hpp"
#include "skill_books/game_skill_book/place_ball.hpp"
#include "skill_books/game_skill_book/intercept_ball_at_point.hpp"
// end of includes DO NOT MODIFY THIS LINE

namespace luhsoccer::skills {

GameSkillBook::GameSkillBook(const config_provider::ConfigStore& cs)
    : SkillBook<GameSkillNames>({
          // add skills after this line (do not delete this line)

          {GameSkillNames::GO_TO_POINT, GoToPointBuild().build(cs)},
          {GameSkillNames::HALT, HaltBuild().build(cs)},
          {GameSkillNames::MARK_ENEMY_LOS_VARIABLE, MarkEnemyLosVariableBuild().build(cs)},
          {GameSkillNames::GO_TO_POINT_WITH_HEADING, GoToPointWithHeadingBuild().build(cs)},
          {GameSkillNames::CONTROLLER_GO_TO_POINT, ControllerGoToPoint().build(cs)},
          {GameSkillNames::MARK_GOALIE_LOS, MarkGoalieLosBuild().build(cs)},
          {GameSkillNames::GOALIE_DEFEND_ON_CIRCLE, GoalieDefendOnCircleBuild().build(cs)},
          {GameSkillNames::GOALIE_LOS_DEFEND, GoalieLosDefendBuild().build(cs)},
          {GameSkillNames::GOALIE_LOS_DEFEND_WO_BALL, GoalieLosDefendWoBallBuild().build(cs)},
          {GameSkillNames::GO_TO_BALL, GoToBallBuild().build(cs)},
          {GameSkillNames::GET_BALL, GetBallBuild().build(cs)},
          {GameSkillNames::KICK_BALL, KickBallBuild().build(cs)},
          {GameSkillNames::MARK_ENEMY_TO_BALL, MarkEnemyToBallBuild().build(cs)},
          {GameSkillNames::GO_TO_PENALTY_LINE, GoToPenaltyLineBuild().build(cs)},
          {GameSkillNames::WALL_AT_PENALTY_AREA, WallAtPenaltyAreaBuild().build(cs)},
          {GameSkillNames::OCTO_SKILL, OctoSkillBuild().build(cs)},
          {GameSkillNames::INTERCEPT_BALL, InterceptBallBuild().build(cs)},
          {GameSkillNames::KICK_REFLEX, KickReflexBuild().build(cs)},
          {GameSkillNames::STEAL_BALL, StealBallBuild().build(cs)},
          {GameSkillNames::DRIBBLE, DribbleBuild().build(cs)},
          {GameSkillNames::PLACE_BALL, PlaceBallBuild().build(cs)},
          {GameSkillNames::INTERCEPT_BALL_AT_POINT, InterceptBallAtPointBuild().build(cs)},
          // do not delete this line either
      }){};

}
