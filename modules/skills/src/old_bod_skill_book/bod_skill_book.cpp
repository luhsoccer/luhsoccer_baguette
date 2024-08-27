#include "skill_books/bod_skill_book.hpp"

#include "config_provider/config_store_main.hpp"

// include skill file here
#include "skill_books/bod_skill_book/go_to_point.hpp"
#include "skill_books/bod_skill_book/get_ball.hpp"
#include "skill_books/bod_skill_book/go_to_point_aligned.hpp"
#include "skill_books/bod_skill_book/halt.hpp"
#include "skill_books/bod_skill_book/mark_enemy_to_ball.hpp"
#include "skill_books/bod_skill_book/block_enemy_line_of_sight.hpp"
#include "skill_books/bod_skill_book/move_to_penalty_line.hpp"
#include "skill_books/bod_skill_book/mark_enemy_to_goal.hpp"
#include "skill_books/bod_skill_book/wall_at_penalty_area.hpp"
#include "skill_books/bod_skill_book/kick_ball_through_target.hpp"
#include "skill_books/bod_skill_book/steal_ball.hpp"
#include "skill_books/bod_skill_book/intercept_ball.hpp"
#include "skill_books/bod_skill_book/intercept_ball_goalie.hpp"
#include "skill_books/bod_skill_book/defend_goal_on_circle.hpp"
#include "skill_books/bod_skill_book/block_goalie_lo_s.hpp"
#include "skill_books/bod_skill_book/reflex_kick.hpp"
#include "skill_books/bod_skill_book/prepare_kick.hpp"
#include "skill_books/bod_skill_book/defend_goalline.hpp"
#include "skill_books/bod_skill_book/wall_at_distance.hpp"
#include "skill_books/bod_skill_book/block_enemy_los_variable.hpp"
#include "skill_books/bod_skill_book/go_to_point_with_heading.hpp"
#include "skill_books/bod_skill_book/move_to_ball_turn_radius.hpp"
#include "skill_books/bod_skill_book/drive_to_line.hpp"
#include "skill_books/bod_skill_book/okto_skill.hpp"
#include "skill_books/bod_skill_book/drive_in_ball.hpp"
#include "skill_books/bod_skill_book/backwards_dribbling.hpp"
#include "skill_books/bod_skill_book/drive_to_line_segment.hpp"
#include "skill_books/bod_skill_book/forwards_dribbling.hpp"
#include "skill_books/bod_skill_book/move_constant.hpp"
#include "skill_books/bod_skill_book/move_constant2.hpp"
#include "skill_books/bod_skill_book/receive_ball_at_point.hpp"
#include "skill_books/bod_skill_book/goalie_los_defend.hpp"
#include "skill_books/bod_skill_book/goalie_los_defend_w_o_ball.hpp"
#include "skill_books/bod_skill_book/intercept_ball_reflex.hpp"
// end of includes DO NOT MODIFY THIS LINE

namespace luhsoccer::skills {

BodSkillBook::BodSkillBook(const config_provider::ConfigStore& cs)
    : SkillBook<BodSkillNames>({
          // add skills after this line (do not delete this line)
          {BodSkillNames::GO_TO_POINT, GoToPointBuild().build(cs)},
          {BodSkillNames::GET_BALL, GetBallBuild().build(cs)},
          {BodSkillNames::GO_TO_POINT_ALIGNED, goToPointAlignedBuild().build(cs)},
          {BodSkillNames::HALT, HaltBuild().build(cs)},
          {BodSkillNames::MARK_ENEMY_TO_BALL, MarkEnemyToBallBuild().build(cs)},
          {BodSkillNames::MARK_ENEMY_TO_GOAL, MarkEnemyToGoalBuild().build(cs)},
          {BodSkillNames::BLOCK_ENEMY_LINE_OF_SIGHT, BlockEnemyLineOfSightBuild().build(cs)},
          {BodSkillNames::MOVE_TO_PENALTY_LINE, MoveToPenaltyLineBuild().build(cs)},
          {BodSkillNames::WALL_AT_PENALTY_AREA, WallAtPenaltyAreaBuild().build(cs)},
          {BodSkillNames::KICK_BALL_THROUGH_TARGET, KickBallThroughTargetBuild().build(cs)},
          {BodSkillNames::STEAL_BALL, StealBallBuild().build(cs)},
          {BodSkillNames::INTERCEPT_BALL, InterceptBallBuild().build(cs)},
          {BodSkillNames::INTERCEPT_BALL_GOALIE, InterceptBallGoalieBuild().build(cs)},
          {BodSkillNames::INTERCEPT_BALL_REFLEX, InterceptBallReflexBuild().build(cs)},
          {BodSkillNames::DEFEND_GOAL_ON_CIRCLE, DefendGoalOnCircleBuild().build(cs)},
          {BodSkillNames::BLOCK_GOALIE_LO_S, BlockGoalieLoSBuild().build(cs)},
          {BodSkillNames::REFLEX_KICK, ReflexKickBuild().build(cs)},
          {BodSkillNames::WALL_AT_DISTANCE, WallAtDistanceBuild().build(cs)},
          {BodSkillNames::PREPARE_KICK, PrepareKickBuild().build(cs)},
          {BodSkillNames::DEFEND_GOALLINE, DefendGoallineBuild().build(cs)},
          {BodSkillNames::BLOCK_ENEMY_LOS_VARIABLE, BlockEnemyLosVariableBuild().build(cs)},
          {BodSkillNames::GO_TO_POINT_WITH_HEADING, GoToPointWithHeadingBuild().build(cs)},
          {BodSkillNames::MOVE_TO_BALL_TURN_RADIUS, MoveToBallTurnRadiusBuild().build(cs)},
          {BodSkillNames::DRIVE_TO_LINE, DriveToLineBuild().build(cs)},
          {BodSkillNames::OKTO_SKILL, OktoSkillBuild().build(cs)},
          {BodSkillNames::DRIVE_IN_BALL, DriveInBallBuild().build(cs)},
          {BodSkillNames::BACKWARDS_DRIBBLING, BackwardsDribblingBuild().build(cs)},
          {BodSkillNames::DRIVE_TO_LINE_SEGMENT, DriveToLineSegmentBuild().build(cs)},
          {BodSkillNames::FORWARDS_DRIBBLING, ForwardsDribblingBuild().build(cs)},
          {BodSkillNames::MOVE_CONSTANT, MoveConstantBuild().build(cs)},
          {BodSkillNames::MOVE_CONSTANT2, MoveConstant2Build().build(cs)},
          {BodSkillNames::RECEIVE_BALL_AT_POINT, ReceiveBallAtPointBuild().build(cs)},
          {BodSkillNames::GOALIE_LOS_DEFEND, goalieLosDefendBuild().build(cs)},
          {BodSkillNames::GOALIE_LOS_DEFEND_W_O_BALL, GoalieLosDefendWOBallBuild().build(cs)},
          // do not delete this line either
      }){};

}
