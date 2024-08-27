#include <gtest/gtest.h>

#include "local_planner/skills/task.hpp"
#include "local_planner_components/steps/condition_step.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/steps/wait_step.hpp"
#include "../world_model_test_helper.hpp"
#include "local_planner/avoidance_manager.hpp"
namespace luhsoccer::local_planner {
TEST(condition_step, general) {
    std::string observe_frame = "observe_frame";
    std::shared_ptr<transform::WorldModel> wm = std::make_shared<transform::WorldModel>(observe_frame);
    std::shared_ptr<AvoidanceManager> avoidance_manager = nullptr;
    local_planner::TaskData td(wm->getPossibleRobots().at(0));

    DoubleComponentParam duration_in_seconds(1.0);

    BoolComponentParam cond(false);
    ConditionStep c = ConditionStep(cond);
    c.addIfStep(WaitStep(WAIT_DURATION, duration_in_seconds));

    EXPECT_EQ(c.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(0)).first,
              AbstractStep::StepState::FINISHED);

    cond = BoolComponentParam(true);
    ConditionStep c1 = ConditionStep(cond);
    c1.addIfStep(WaitStep(WAIT_DURATION, duration_in_seconds));

    EXPECT_EQ(
        c1.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(.4)).first,
        AbstractStep::StepState::RUNNING);
    EXPECT_EQ(
        c1.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(5.0)).first,
        AbstractStep::StepState::FINISHED);

    cond = BoolComponentParam(true);
    ConditionStep c2 = ConditionStep(cond);
    EXPECT_EQ(c2.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager).first,
              AbstractStep::StepState::FINISHED);
};

TEST(condition_step, if_part) {
    DoubleComponentParam duration_in_seconds(1.0);
    std::string observe_frame = "observe_frame";
    std::shared_ptr<AvoidanceManager> avoidance_manager = nullptr;
    std::shared_ptr<transform::WorldModel> wm = std::make_shared<transform::WorldModel>(observe_frame);
    local_planner::TaskData td(wm->getPossibleRobots().at(0));

    BoolComponentParam cond = BoolComponentParam(true);
    ConditionStep c3 = ConditionStep(cond);
    c3.addIfStep(WaitStep(WAIT_DURATION, duration_in_seconds));
    c3.addIfStep(DribblerStep(robot_interface::DribblerMode::HIGH));
    c3.addIfStep(DribblerStep(robot_interface::DribblerMode::HIGH));
    c3.addIfStep(WaitStep(WAIT_DURATION, duration_in_seconds));

    EXPECT_EQ(
        c3.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(0.5)).first,
        AbstractStep::StepState::RUNNING);
    EXPECT_EQ(
        c3.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(3.0)).first,
        AbstractStep::StepState::RUNNING);
    EXPECT_EQ(c3.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager).first,
              AbstractStep::StepState::RUNNING);
    EXPECT_EQ(c3.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager).first,
              AbstractStep::StepState::RUNNING);
    EXPECT_EQ(
        c3.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(0.8)).first,
        AbstractStep::StepState::RUNNING);
    EXPECT_EQ(
        c3.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(7.0)).first,
        AbstractStep::StepState::FINISHED);
    EXPECT_EQ(c3.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager).first,
              AbstractStep::StepState::FINISHED);
};

TEST(condition_step, else_part) {
    DoubleComponentParam duration_in_seconds(1.0);
    std::string observe_frame = "observe_frame";
    std::shared_ptr<AvoidanceManager> avoidance_manager = nullptr;
    std::shared_ptr<transform::WorldModel> wm = std::make_shared<transform::WorldModel>(observe_frame);
    local_planner::TaskData td(wm->getPossibleRobots().at(0));

    BoolComponentParam cond = BoolComponentParam(false);
    ConditionStep c4 = ConditionStep(cond);
    c4.addElseStep(WaitStep(WAIT_DURATION, duration_in_seconds));
    c4.addElseStep(DribblerStep(robot_interface::DribblerMode::HIGH));
    c4.addElseStep(DribblerStep(robot_interface::DribblerMode::HIGH));
    c4.addElseStep(WaitStep(WAIT_DURATION, duration_in_seconds));

    EXPECT_EQ(
        c4.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(0.5)).first,
        AbstractStep::StepState::RUNNING);
    EXPECT_EQ(
        c4.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(3.0)).first,
        AbstractStep::StepState::RUNNING);
    EXPECT_EQ(c4.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager).first,
              AbstractStep::StepState::RUNNING);
    EXPECT_EQ(c4.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager).first,
              AbstractStep::StepState::RUNNING);
    EXPECT_EQ(
        c4.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(0.8)).first,
        AbstractStep::StepState::RUNNING);
    EXPECT_EQ(
        c4.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(7.0)).first,
        AbstractStep::StepState::FINISHED);
    EXPECT_EQ(c4.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager).first,
              AbstractStep::StepState::FINISHED);
};
}  // namespace luhsoccer::local_planner