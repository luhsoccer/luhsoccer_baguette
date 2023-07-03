#include "bindings.hpp"

namespace luhsoccer::python {

using namespace skills;

template <>
void bindEnum(py::enum_<BodSkillNames>& instance) {
    // @todo add a way to create the names dynamically
    instance.value("GoToPoint", BodSkillNames::GO_TO_POINT);
    instance.value("GoToPointAligned", BodSkillNames::GO_TO_POINT_ALIGNED);
    instance.value("GetBall", BodSkillNames::GET_BALL);
    instance.value("Halt", BodSkillNames::HALT);
    instance.value("MarkEnemyToBall", BodSkillNames::MARK_ENEMY_TO_BALL);
    instance.value("BlockEnemyLineOfSight", BodSkillNames::BLOCK_ENEMY_LINE_OF_SIGHT);
    instance.value("MoveToPenaltyLine", BodSkillNames::MOVE_TO_PENALTY_LINE);
    instance.value("MarkEnemyToGoal", BodSkillNames::MARK_ENEMY_TO_GOAL);
    instance.value("KickBallThroughTarget", BodSkillNames::KICK_BALL_THROUGH_TARGET);
    instance.value("KickBallToTarget", BodSkillNames::KICK_BALL_TO_TARGET);
    instance.value("WallAtPenaltyArea", BodSkillNames::WALL_AT_PENALTY_AREA);
    instance.value("StealBall", BodSkillNames::STEAL_BALL);
    instance.value("ReceiveBall", BodSkillNames::RECEIVE_BALL);
    instance.value("DefendGoalOnCircle", BodSkillNames::DEFEND_GOAL_ON_CIRCLE);
    instance.value("BlockGoalieLoS", BodSkillNames::BLOCK_GOALIE_LO_S);
    instance.value("ReflexKick", BodSkillNames::REFLEX_KICK);
    instance.value("PassBallToRobot", BodSkillNames::PASS_BALL_TO_ROBOT);
    instance.value("PrepareKick", BodSkillNames::PREPARE_KICK);
    instance.value("DefendGoalline", BodSkillNames::DEFEND_GOALLINE);
    instance.value("WallAtDistance", BodSkillNames::WALL_AT_DISTANCE);
    instance.value("BlockEnemyLosVariable", BodSkillNames::BLOCK_ENEMY_LOS_VARIABLE);
    instance.value("GoToPointWithHeading", BodSkillNames::GO_TO_POINT_WITH_HEADING);
    instance.value("MoveToBallTurnRadius", BodSkillNames::MOVE_TO_BALL_TURN_RADIUS);
    instance.value("DriveToLine", BodSkillNames::DRIVE_TO_LINE);
    instance.value("KickBallThroughTargetPreposition", BodSkillNames::KICK_BALL_THROUGH_TARGET_PREPOSITION);
    instance.value("DriveBySide", BodSkillNames::DRIVE_BY_SIDE);
    instance.value("kickBallThroughTargetOriginal", BodSkillNames::KICK_BALL_THROUGH_TARGET_ORIGINAL);
    instance.value("kickBallThroughTargetDirect", BodSkillNames::KICK_BALL_THROUGH_TARGET_DIRECT);
    instance.value("RunFree", BodSkillNames::RUN_FREE);
    instance.value("OktoSkill", BodSkillNames::OKTO_SKILL);
    instance.value("DriveInBall", BodSkillNames::DRIVE_IN_BALL);
    instance.value("GoToPointOriginal", BodSkillNames::GO_TO_POINT_ORIGINAL);
    instance.value("GoToPointWithHeadingOriginal", BodSkillNames::GO_TO_POINT_WITH_HEADING_ORIGINAL);
    instance.value("GoToPointDribbler", BodSkillNames::GO_TO_POINT_DRIBBLER);
    instance.value("BackwardsDribbling", BodSkillNames::BACKWARDS_DRIBBLING);
    instance.value("DriveToLineSegment", BodSkillNames::DRIVE_TO_LINE_SEGMENT);
    instance.value("GoToPointIntercept", BodSkillNames::GO_TO_POINT_INTERCEPT);
    instance.value("ForwardsDribbling", BodSkillNames::FORWARDS_DRIBBLING);
}  // DO NOT EDIT THIS LINE

template <>
void bindClass(py::class_<local_planner::Skill>& instance) {
    instance.def("__repr__", [](const local_planner::Skill& object) { return object.name; });
}

template <>
void bindClass(py::class_<BodSkillBook>& instance) {
    instance.def("getSkill", &BodSkillBook::getSkill, py::return_value_policy::reference);
}

}  // namespace luhsoccer::python