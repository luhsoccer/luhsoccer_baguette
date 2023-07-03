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
#include "skill_books/bod_skill_book/kick_ball_to_target.hpp"
#include "skill_books/bod_skill_book/steal_ball.hpp"
#include "skill_books/bod_skill_book/receive_ball.hpp"
#include "skill_books/bod_skill_book/defend_goal_on_circle.hpp"
#include "skill_books/bod_skill_book/block_goalie_lo_s.hpp"
#include "skill_books/bod_skill_book/pass_ball_to_robot.hpp"
#include "skill_books/bod_skill_book/reflex_kick.hpp"
#include "skill_books/bod_skill_book/prepare_kick.hpp"
#include "skill_books/bod_skill_book/defend_goalline.hpp"
#include "skill_books/bod_skill_book/wall_at_distance.hpp"
#include "skill_books/bod_skill_book/block_enemy_los_variable.hpp"
#include "skill_books/bod_skill_book/go_to_point_with_heading.hpp"
#include "skill_books/bod_skill_book/move_to_ball_turn_radius.hpp"
#include "skill_books/bod_skill_book/drive_to_line.hpp"
#include "skill_books/bod_skill_book/kick_ball_through_target_preposition.hpp"
#include "skill_books/bod_skill_book/drive_by_side.hpp"
#include "skill_books/bod_skill_book/kick_ball_through_target_original.hpp"
#include "skill_books/bod_skill_book/kick_ball_through_target_direct.hpp"
#include "skill_books/bod_skill_book/run_free.hpp"
#include "skill_books/bod_skill_book/okto_skill.hpp"
#include "skill_books/bod_skill_book/drive_in_ball.hpp"
#include "skill_books/bod_skill_book/go_to_point_original.hpp"
#include "skill_books/bod_skill_book/go_to_point_with_heading_original.hpp"
#include "skill_books/bod_skill_book/go_to_point_dribbler.hpp"
#include "skill_books/bod_skill_book/backwards_dribbling.hpp"
#include "skill_books/bod_skill_book/drive_to_line_segment.hpp"
#include "skill_books/bod_skill_book/go_to_point_intercept.hpp"
#include "skill_books/bod_skill_book/forwards_dribbling.hpp"
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
          {BodSkillNames::KICK_BALL_TO_TARGET, KickBallToTargetBuild().build(cs)},
          {BodSkillNames::STEAL_BALL, StealBallBuild().build(cs)},
          {BodSkillNames::RECEIVE_BALL, ReceiveBallBuild().build(cs)},
          {BodSkillNames::DEFEND_GOAL_ON_CIRCLE, DefendGoalOnCircleBuild().build(cs)},
          {BodSkillNames::BLOCK_GOALIE_LO_S, BlockGoalieLoSBuild().build(cs)},
          {BodSkillNames::PASS_BALL_TO_ROBOT, passBallToRobotBuild().build(cs)},
          {BodSkillNames::REFLEX_KICK, ReflexKickBuild().build(cs)},
          {BodSkillNames::WALL_AT_DISTANCE, WallAtDistanceBuild().build(cs)},
          {BodSkillNames::PREPARE_KICK, PrepareKickBuild().build(cs)},
          {BodSkillNames::DEFEND_GOALLINE, DefendGoallineBuild().build(cs)},
          {BodSkillNames::BLOCK_ENEMY_LOS_VARIABLE, BlockEnemyLosVariableBuild().build(cs)},
          {BodSkillNames::GO_TO_POINT_WITH_HEADING, GoToPointWithHeadingBuild().build(cs)},
          {BodSkillNames::MOVE_TO_BALL_TURN_RADIUS, MoveToBallTurnRadiusBuild().build(cs)},
          {BodSkillNames::DRIVE_TO_LINE, DriveToLineBuild().build(cs)},
          {BodSkillNames::KICK_BALL_THROUGH_TARGET_PREPOSITION, KickBallThroughTargetPrepositionBuild().build(cs)},
          {BodSkillNames::DRIVE_BY_SIDE, DriveBySideBuild().build(cs)},
          {BodSkillNames::KICK_BALL_THROUGH_TARGET_ORIGINAL, kickBallThroughTargetOriginalBuild().build(cs)},
          {BodSkillNames::KICK_BALL_THROUGH_TARGET_DIRECT, kickBallThroughTargetDirectBuild().build(cs)},
          {BodSkillNames::RUN_FREE, RunFreeBuild().build(cs)},
          {BodSkillNames::OKTO_SKILL, OktoSkillBuild().build(cs)},
          {BodSkillNames::DRIVE_IN_BALL, DriveInBallBuild().build(cs)},
          {BodSkillNames::GO_TO_POINT_ORIGINAL, GoToPointOriginalBuild().build(cs)},
          {BodSkillNames::GO_TO_POINT_WITH_HEADING_ORIGINAL, GoToPointWithHeadingOriginalBuild().build(cs)},
          {BodSkillNames::GO_TO_POINT_DRIBBLER, GoToPointDribblerBuild().build(cs)},
          {BodSkillNames::BACKWARDS_DRIBBLING, BackwardsDribblingBuild().build(cs)},
          {BodSkillNames::DRIVE_TO_LINE_SEGMENT, DriveToLineSegmentBuild().build(cs)},
          {BodSkillNames::GO_TO_POINT_INTERCEPT, GoToPointInterceptBuild().build(cs)},
          {BodSkillNames::FORWARDS_DRIBBLING, ForwardsDribblingBuild().build(cs)},
          // do not delete this line either
      }){};

}
