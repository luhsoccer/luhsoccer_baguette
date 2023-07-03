
#include "config_provider/config_store_main.hpp"
#include "local_planner/local_planner_module.hpp"
#include "local_planner/avoidance_manager.hpp"

namespace luhsoccer::local_planner {

LocalPlannerModule::LocalPlannerModule(const std::shared_ptr<const transform::WorldModel>& wm,
                                       robot_interface::RobotInterface& robot_interface, marker::MarkerService& ms)
    : cs(config_provider::ConfigProvider::getConfigStore()),
      wm(wm),
      robot_interface(robot_interface),
      ms(ms),
      avoidance_manager(std::make_shared<AvoidanceManager>()),
      simulation_manager(this->thread_pool, ms, wm, avoidance_manager),
      logger("local_planner_module") {}

void LocalPlannerModule::setup() {
    std::vector<RobotIdentifier> robots = this->wm->getPossibleRobots<Team::ALLY>();

    for (const RobotIdentifier& robot : robots) {
        LOG_DEBUG(this->logger, "Creating local planner for {} with frequency {:d}Hz", robot,
                  cs.local_planner_components_config.controller_freq.val());
        this->planners.emplace(std::piecewise_construct, std::forward_as_tuple(robot),
                               std::forward_as_tuple(this->wm, this->avoidance_manager, this->robot_interface, this->ms,
                                                     robot, cs.local_planner_components_config.controller_freq.val()));
    }
}

void LocalPlannerModule::loop(std::atomic_bool& should_run) {
    // start loop of all planners
    for (auto& [robot, planner] : this->planners) {
        if (!planner.startLoop(should_run)) {
            LOG_ERROR(this->logger, "The LocalPlanner of {} could not be started. Is the planner already running?",
                      robot);
        }
    }
    this->thread_pool.start(should_run);

    // block until module should shutdown
    time::Rate rate(2, "LocalPlannerModule/IdleLoop");
    while (should_run) {
        updateAvoidManagerModes();
        rate.sleep();
    }
}

void LocalPlannerModule::stop() {
    this->simulation_manager.stopSimulations();
    LOG_DEBUG(this->logger, "Waiting for planners to finish");
    for (auto& [robot, planner] : this->planners) {
        planner.join();
    }
    LOG_DEBUG(this->logger, "All planners have finished");
}

bool LocalPlannerModule::setTask(const Skill* skill, const TaskData& td) {
    try {
        if (!this->planners.at(td.robot).setTask(skill, td)) return false;
        // this->simulation_manager.startNewTask(this->planners.at(td.robot), td.robot, this->wm, skill, td);
        // this->visualizeTask(skill, td);
        return true;
    } catch (std::out_of_range& e) {
        LOG_WARNING(this->logger, "No planner exists for requested {}. Task could not be set!.", td.robot);
        return false;
    }
}

bool LocalPlannerModule::cancelTask(const RobotIdentifier& robot) {
    try {
        return this->planners.at(robot).cancelCurrentTask();
    } catch (std::out_of_range& e) {
        LOG_WARNING(this->logger, "No planner exists for requested {}. Task could not be canceled!.", robot);
        return false;
    }
}

std::optional<LocalPlanner::LocalPlannerState> LocalPlannerModule::getState(const RobotIdentifier& robot) const {
    auto planner = this->planners.find(robot);
    if (planner == this->planners.end()) return std::nullopt;

    return planner->second.getState();
}

void LocalPlannerModule::setState(bool online) {
    for (auto& [key, planner] : this->planners) {
        planner.setState(online);
    }
}

bool LocalPlannerModule::setStateOfPlanner(RobotIdentifier robot, bool online) {
    auto robot_it = this->planners.find(robot);
    if (robot_it == this->planners.end()) return false;
    robot_it->second.setState(online);
    return true;
}

void LocalPlannerModule::updateAvoidManagerModes() {
    static const std::map<std::string, AvoidForceMode> AVOID_FORCE_MODE = {
        {"CIRCULAR_FIELDS", AvoidForceMode::CIRCULAR_FIELDS},
        {"APF", AvoidForceMode::APF},
        {"ATAKA", AvoidForceMode::ATAKA},
        {"GYROSCOPIC", AvoidForceMode::GYROSCOPIC},
    };
    static const std::vector<AvoidForceMode> AVOID_FORCE_FROM_NUMBER = {
        AvoidForceMode::CIRCULAR_FIELDS, AvoidForceMode::APF, AvoidForceMode::ATAKA, AvoidForceMode::GYROSCOPIC};

    static const std::map<std::string, MagenticFieldVectorMode> MAGNETIC_FIELD_VECTOR_MAPPING = {
        {"OFF", MagenticFieldVectorMode::OFF},
        {"MULTI_AGENT", MagenticFieldVectorMode::MULTI_AGENT},
        {"SIDE_DECISION", MagenticFieldVectorMode::SIDE_DECISION},
        {"COOPERATIVE_SIDE_DECISION", MagenticFieldVectorMode::COOPERATIVE_SIDE_DECISION},
        {"HADDADIN", MagenticFieldVectorMode::HADDADIN}};

    static const std::vector<MagenticFieldVectorMode> MAGNETIC_FIELD_VECTOR_FROM_NUMBER = {
        MagenticFieldVectorMode::OFF, MagenticFieldVectorMode::MULTI_AGENT, MagenticFieldVectorMode::SIDE_DECISION,
        MagenticFieldVectorMode::COOPERATIVE_SIDE_DECISION, MagenticFieldVectorMode::HADDADIN};
    // auto avoid_force_mode_it = AVOID_FORCE_MODE.find(localPlannerConfig().avoid_force_mode);
    // if (avoid_force_mode_it != AVOID_FORCE_MODE.end()) {
    //     this->avoidance_manager->setAvoidForceMode(avoid_force_mode_it->second);
    // } else {
    //     LOG_WARNING(this->logger, "avoid_force_mode '{}' is not known!",
    //     localPlannerConfig().avoid_force_mode.val());
    // }
    // auto magnetic_field_vector_mode =
    //     MAGNETIC_FIELD_VECTOR_MAPPING.find(localPlannerConfig().magentic_field_vector_mode);
    // if (magnetic_field_vector_mode != MAGNETIC_FIELD_VECTOR_MAPPING.end()) {
    //     this->avoidance_manager->setMagenticFieldVectorMode(magnetic_field_vector_mode->second);
    // } else {
    //     LOG_WARNING(this->logger, "magentic_field_vector_mode '{}' is not known!",
    //                 localPlannerConfig().magentic_field_vector_mode.val());
    // }
    this->avoidance_manager->setAvoidForceMode(AVOID_FORCE_FROM_NUMBER[localPlannerConfig().avoid_force_mode_num]);
    this->avoidance_manager->setMagenticFieldVectorMode(
        MAGNETIC_FIELD_VECTOR_FROM_NUMBER[localPlannerConfig().magentic_field_vector_mode_num]);
}

void LocalPlannerModule::visualizeTask(const Skill* skill, const TaskData& td) {
    time::TimePoint start_sim = time::now();
    auto result_callback = [this, skill, td, start_sim](unsigned long /*id*/, const SimulationResult& result) {
        constexpr time::Duration STEP_SIZE = 0.01;
        marker::LineStrip strip("", fmt::format("{}-SkillPrediction", td.robot), 0);
        std::vector<marker::Point> points;
        for (time::TimePoint t = result.start_time; t < result.end_time; t += STEP_SIZE) {
            auto pos = result.wm->getTransform(td.robot.getFrame(), "", t);
            if (pos.has_value()) {
                points.emplace_back(pos->transform.translation().x(), pos->transform.translation().y(),
                                    Eigen::Rotation2Dd(pos->transform.rotation()).angle());
            }
        }
        if (points.size() <= 2) return;
        constexpr double SIMULATION_MARKER_LIFETIME = 5.0;
        strip.setPoints(points);
        strip.setColor(marker::Color::BLUE());
        strip.setLifetime(SIMULATION_MARKER_LIFETIME);
        this->ms.displayMarker(strip);

        marker::Circle target({"", points.back().x, points.back().y}, fmt::format("{}-SkillPrediction", td.robot), 1);
        constexpr double TARGET_MARKER_RADIUS = 0.05;
        target.setRadius(TARGET_MARKER_RADIUS);
        target.setColor(marker::Color::RED());
        target.setLifetime(SIMULATION_MARKER_LIFETIME);
        this->ms.displayMarker(target);

        time::Duration time_sim = time::now() - start_sim;
        LOG_INFO(this->logger, "Simulation of skill '{}' for {} took {:d}ms. Skill takes {:0.3f}s to execute.",
                 skill->name, td.robot, static_cast<int>(time_sim.asSec() * 1000),
                 time::Duration(result.end_time - result.start_time).asSec());
    };
    this->simulation_manager.startSkillSimulation(td.robot, *skill, td, result_callback);
}

}  // namespace luhsoccer::local_planner