#include <gtest/gtest.h>

#include "local_planner/skills/task.hpp"
#include "local_planner_components/steps/wait_step.hpp"
#include "../world_model_test_helper.hpp"
#include "local_planner/avoidance_manager.hpp"

namespace luhsoccer::local_planner {
TEST(wait_step, DurationInSeconds) {
    DoubleComponentParam duration_in_seconds(1.0);
    WaitStep w = WaitStep(WAIT_DURATION, duration_in_seconds);

    std::string observe_frame = "observe_frame";
    std::shared_ptr<transform::WorldModel> wm = std::make_shared<transform::WorldModel>(observe_frame);
    std::shared_ptr<AvoidanceManager> avoidance_manager = nullptr;
    local_planner::TaskData td(wm->getPossibleRobots().at(0));

    ASSERT_EQ(w.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(.1)).first,
              AbstractStep::StepState::RUNNING);

    ASSERT_EQ(
        w.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(.85)).first,
        AbstractStep::StepState::RUNNING);

    ASSERT_EQ(
        w.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(1.100001)).first,
        AbstractStep::StepState::FINISHED);
};

TEST(wait_step, CheckBool) {
    config_provider::ConfigStore cs;

    bool wait_bool = true;

    BoolComponentParam wait(
        CALLBACK, [&wait_bool](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> bool {
            return wait_bool;
        });

    WaitStep w = WaitStep(WAIT_BOOL, wait);

    std::string observe_frame = "observe_frame";
    std::shared_ptr<transform::WorldModel> wm = std::make_shared<transform::WorldModel>(observe_frame);
    std::shared_ptr<AvoidanceManager> avoidance_manager = nullptr;

    local_planner::TaskData td(wm->getPossibleRobots().at(0));

    ASSERT_EQ(w.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(.1)).first,
              AbstractStep::StepState::RUNNING);

    wait_bool = false;
    ASSERT_EQ(
        w.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(.85)).first,
        AbstractStep::StepState::FINISHED);
};

TEST(wait_step, Timepoint) {
    time::TimePoint timepoint(time::TimePoint(1.0));
    WaitStep w = WaitStep(WAIT_UNTIL_TIME_POINT, timepoint);

    std::string observe_frame = "observe_frame";
    std::shared_ptr<transform::WorldModel> wm = std::make_shared<transform::WorldModel>(observe_frame);
    std::shared_ptr<AvoidanceManager> avoidance_manager = nullptr;

    local_planner::TaskData td(wm->getPossibleRobots().at(0));

    ASSERT_EQ(w.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(.1)).first,
              AbstractStep::StepState::RUNNING);

    ASSERT_EQ(
        w.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(1.1)).first,
        AbstractStep::StepState::FINISHED);
}

TEST(wait_step, PositionDistanceTravelled) {
    WaitStep w = WaitStep(transform::Position("observe_frame", 0.0, 0.0), 0.5);

    std::string observe_frame = "observe_frame";
    std::shared_ptr<transform::WorldModel> wm = std::make_shared<transform::WorldModel>();
    std::shared_ptr<AvoidanceManager> avoidance_manager = nullptr;

    local_planner::TaskData td(wm->getPossibleRobots().at(0));
    transform::TransformWithVelocity transform;
    transform.transform = Eigen::Translation2d(0.0, 0.0) * Eigen::Rotation2Dd(0.0);
    transform.header.stamp = time::TimePoint(.1);
    transform.header.child_frame = observe_frame;
    transform.header.parent_frame = "";
    wm->pushTransform(transform, true);

    ASSERT_EQ(w.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(.1)).first,
              AbstractStep::StepState::RUNNING);

    transform.transform = Eigen::Translation2d(1.0, 0.0) * Eigen::Rotation2Dd(0.0);
    transform.header.stamp = time::TimePoint(.2);
    wm->pushTransform(transform, true);
    ASSERT_EQ(w.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(.3)).first,
              AbstractStep::StepState::FINISHED);
}

TEST(wait_step, PositionDistanceTravelledTimeout) {
    WaitStep w = WaitStep(transform::Position("observe_frame", 0.0, 0.0), 0.5, 0.4);

    std::string observe_frame = "observe_frame";
    std::shared_ptr<transform::WorldModel> wm = std::make_shared<transform::WorldModel>();
    std::shared_ptr<AvoidanceManager> avoidance_manager = nullptr;

    local_planner::TaskData td(wm->getPossibleRobots().at(0));
    transform::TransformWithVelocity transform;
    transform.transform = Eigen::Translation2d(0.0, 0.0) * Eigen::Rotation2Dd(0.0);
    transform.header.stamp = time::TimePoint(.1);
    transform.header.child_frame = observe_frame;
    transform.header.parent_frame = "";
    wm->pushTransform(transform, true);

    ASSERT_EQ(w.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(.2)).first,
              AbstractStep::StepState::RUNNING);

    transform.transform = Eigen::Translation2d(0.4, 0.0) * Eigen::Rotation2Dd(0.0);
    transform.header.stamp = time::TimePoint(.3);
    wm->pushTransform(transform, true);

    ASSERT_EQ(
        w.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(0.8)).first,
        AbstractStep::StepState::FINISHED);
}

TEST(wait_step, DistanceBetweenPositions) {
    WaitStep w =
        WaitStep(transform::Position("", 0.0, 0.0), transform::Position("observe_frame", 0.0, 0.0), 0.5, false);

    std::string observe_frame = "observe_frame";
    std::shared_ptr<transform::WorldModel> wm = std::make_shared<transform::WorldModel>();
    std::shared_ptr<AvoidanceManager> avoidance_manager = nullptr;

    local_planner::TaskData td(wm->getPossibleRobots().at(0));
    transform::TransformWithVelocity transform;
    transform.transform = Eigen::Translation2d(0.0, 0.0) * Eigen::Rotation2Dd(0.0);
    transform.header.stamp = time::TimePoint(.1);
    transform.header.child_frame = observe_frame;
    transform.header.parent_frame = "";
    wm->pushTransform(transform, true);

    ASSERT_EQ(w.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(.2)).first,
              AbstractStep::StepState::RUNNING);

    transform.transform = Eigen::Translation2d(1.0, 0.0) * Eigen::Rotation2Dd(0.0);
    transform.header.stamp = time::TimePoint(.3);
    wm->pushTransform(transform, true);

    ASSERT_EQ(w.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(.4)).first,
              AbstractStep::StepState::FINISHED);
}

TEST(wait_step, DistanceBetweenPositionsTimeout) {
    WaitStep w =
        WaitStep(transform::Position("", 0.0, 0.0), transform::Position("observe_frame", 0.0, 0.0), 0.5, false, 0.4);

    std::string observe_frame = "observe_frame";
    std::shared_ptr<transform::WorldModel> wm = std::make_shared<transform::WorldModel>();
    std::shared_ptr<AvoidanceManager> avoidance_manager = nullptr;

    local_planner::TaskData td(wm->getPossibleRobots().at(0));
    transform::TransformWithVelocity transform;
    transform.transform = Eigen::Translation2d(0.0, 0.0) * Eigen::Rotation2Dd(0.0);
    transform.header.stamp = time::TimePoint(.1);
    transform.header.child_frame = observe_frame;
    transform.header.parent_frame = "";
    wm->pushTransform(transform, true);

    ASSERT_EQ(w.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(.2)).first,
              AbstractStep::StepState::RUNNING);

    transform.transform = Eigen::Translation2d(0.2, 0.0) * Eigen::Rotation2Dd(0.0);
    transform.header.stamp = time::TimePoint(.3);
    wm->pushTransform(transform, true);

    ASSERT_EQ(w.calcCommandMessage(wm, td, wm->getPossibleRobots().at(0), avoidance_manager, time::TimePoint(.8)).first,
              AbstractStep::StepState::FINISHED);
}

TEST(wait_step, WaitForBallInDribbler) {
    WaitStep w = WaitStep(WAIT_FOR_BALL, true, 0.5);

    std::string observe_frame = "observe_frame";
    std::shared_ptr<transform::WorldModel> wm = std::make_shared<transform::WorldModel>();
    std::shared_ptr<AvoidanceManager> avoidance_manager = nullptr;

    auto robot = wm->getPossibleRobots().at(0);
    local_planner::TaskData td(robot);

    ASSERT_EQ(w.calcCommandMessage(wm, td, robot, avoidance_manager).first, AbstractStep::StepState::ERROR);

    transform::AllyRobotData data;
    data.ball_in_dribbler = false;
    data.time = time::now();
    wm->pushAllyRobotData(robot, data);

    ASSERT_EQ(w.calcCommandMessage(wm, td, robot, avoidance_manager).first, AbstractStep::StepState::RUNNING);

    data.ball_in_dribbler = true;
    data.time = time::now();
    wm->pushAllyRobotData(robot, data);

    ASSERT_EQ(w.calcCommandMessage(wm, td, robot, avoidance_manager).first, AbstractStep::StepState::FINISHED);
}

TEST(wait_step, WaitForBallInDribblerTimeout) {
    WaitStep w = WaitStep(WAIT_FOR_BALL, true, 0.5);

    std::string observe_frame = "observe_frame";
    std::shared_ptr<AvoidanceManager> avoidance_manager = nullptr;
    std::shared_ptr<transform::WorldModel> wm = std::make_shared<transform::WorldModel>();
    auto robot = wm->getPossibleRobots().at(0);
    local_planner::TaskData td(robot);

    transform::AllyRobotData data;
    data.ball_in_dribbler = false;
    data.time = time::TimePoint(0.1);
    wm->pushAllyRobotData(robot, data);

    ASSERT_EQ(w.calcCommandMessage(wm, td, robot, avoidance_manager, time::TimePoint(.2)).first,
              AbstractStep::StepState::RUNNING);

    ASSERT_EQ(w.calcCommandMessage(wm, td, robot, avoidance_manager, time::TimePoint(.3)).first,
              AbstractStep::StepState::RUNNING);

    ASSERT_EQ(w.calcCommandMessage(wm, td, robot, avoidance_manager, time::TimePoint(.8)).first,
              AbstractStep::StepState::FINISHED);
}

}  // namespace luhsoccer::local_planner
