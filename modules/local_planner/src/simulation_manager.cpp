#include "local_planner/simulation_manager.hpp"
#include "local_planner/thread_pool.hpp"
#include "marker_service/marker_service.hpp"
#include "local_planner/local_planner.hpp"

#include "local_planner/skills/abstract_shape.hpp"

#define SHOW_COLLIED_SIMULATIONS
namespace luhsoccer::local_planner {

// SimulationManager* simulation_manager{};

SimulationManager::SimulationManager(ThreadPool& thread_pool, marker::MarkerService& ms,
                                     const std::shared_ptr<const transform::WorldModel>& real_wm,
                                     const std::shared_ptr<AvoidanceManager>& am)
    : thread_pool(thread_pool), ms(ms), real_wm(real_wm), am(am) {
    // if (!simulation_manager) {
    //     simulation_manager = this;
    // }
}

SimulationManager::~SimulationManager() {
    this->should_run = false;
    for (const auto& simulator : simulators) {
        for (const auto& sim : simulator.second) {
            sim->stopSimulation();
        }
    }
}

void SimulationManager::setNewTask(const RobotIdentifier& robot, const Skill* skill, const TaskData& td) {
    std::lock_guard lock(this->simulator_mtx);
    auto simulator_it = this->simulators.find(robot);

    if (simulator_it != this->simulators.end()) {
        // stop old simulation
        for (auto& simulator : simulator_it->second) {
            simulator->stopSimulation();
        }
        simulator_it->second.clear();
        this->simulators.erase(robot);
    } else {
        this->simulators.emplace(robot, std::vector<std::shared_ptr<Simulator>>());
    }

    this->current_skills.insert_or_assign(robot, std::make_tuple(skill, td, time::now()));
}

// void SimulationManager::handleResult(unsigned long id, const SimulationResult& result,
//                                      const time::TimePoint& start_time, const RobotIdentifier& robot,
//                                      LocalPlanner& local_planner) {
// #ifdef MULTI_AGENT
//     int score = SimulationManager::calculateScore(result);
//     std::optional<TaskData> current_td = local_planner.getCurrentTaskData();
//     bool take_result = false;
//     bool reachable = false;
//     if (current_td.has_value()) {
//         std::optional<int> current_score = this->getCookie<int>(current_td.value(), "score");
//         take_result = !current_score.has_value() || current_score.value() < score;
//         // check if robot is still on the simulation
//         auto robot_pos = real_wm->getTransform(robot.getFrame(), "");
//         if (!robot_pos.has_value()) {
//             take_result = false;
//         }
//         constexpr time::Duration SCANNING_INTERVAL = 0.1;
//         for (time::TimePoint time = result.start_time; time < result.end_time; time += SCANNING_INTERVAL) {
//             auto simulation_pos = result.wm->getTransform(robot.getFrame(), "", time);
//             if (simulation_pos.has_value() &&
//                 (robot_pos->transform.translation() - simulation_pos->transform.translation()).norm() <
//                     localPlannerConfig().simulation_max_deviation) {
//                 reachable = true;
//                 break;
//             }
//         }
//         take_result = take_result && reachable;
//     }

//     if (take_result) {
//         auto task_it = result.tasks.find(robot);
//         TaskData new_task_data = task_it->second.task_data;
//         this->setCookie(new_task_data, "score", score);
//         this->setCookie(new_task_data, "simulation", false);
//         if (task_it != result.tasks.end()) {
//             if (!local_planner.updateTaskData(new_task_data)) {
//                 LOG_WARNING(this->logger, "Updating of TaskData for robot {} from simulation {:d} failed.", robot,
//                 id);
//             }
//         }
//     }

// #else
//     bool reachable = true;
//     bool take_result = true;
//     int score = 1;
// #endif
//     // show results
//     LOG_INFO(this->logger, "Simulation with id {:d} finished after {:.0f} ms with a score of {:d}", id,
//              time::Duration(time::now() - start_time).asSec() * 1000, score);
//     constexpr double VISUALIZATION_INTERVAL = 1.0 / 30.0;
//     std::vector<marker::Point> points;
//     for (auto time = result.start_time; time < result.end_time; time += time::Duration(VISUALIZATION_INTERVAL)) {
//         auto pos = result.wm->getTransform(robot.getFrame(), "", time);
//         if (pos.has_value()) {
//             points.emplace_back(pos->transform.translation().x(), pos->transform.translation().y());
//         }
//     }
// #ifndef SHOW_COLLIED_SIMULATIONS
//     if (score == 0) return;
// #endif
//     marker::LineStrip strip("", fmt::format("{}SimulationResult", robot), id);
//     strip.setLifetime(5);
//     strip.setPoints(points);
//     if (score == 0)
//         strip.setColor(marker::Color::YELLOW());
//     else if (reachable)
//         strip.setColor(marker::Color::GREEN());
//     else
//         strip.setColor(marker::Color::GREY());
//     if (points.size() > 1) {
//         this->ms.displayMarker(strip);
//         marker::Text text({"", points[points.size() / 2].x, points[points.size() / 2].y},
//                           fmt::format("{}SimulationResultText", robot), id);
//         text.setText(fmt::format("Score: {:d}", score));
//         text.setScale(marker::ScaleVec3(2));
//         text.setColor(marker::Color::GREEN());
//         text.setLifetime(5);
//         this->ms.displayMarker(text);
//     }

//     if (take_result) {
//         marker::LineStrip strip2("", fmt::format("{}SimulationResult", robot), 0);
//         strip2.setLifetime(30);
//         strip2.setPoints(points);
//         strip2.setColor(marker::Color::BLUE());
//         if (points.size() > 1) this->ms.displayMarker(strip2);
//     }
// }

void SimulationManager::startTaskDataSimulation(const RobotIdentifier& robot, const TaskData& td,
                                                const ResultCallback& result_callback,
                                                std::shared_ptr<const transform::WorldModel> wm,
                                                const std::optional<time::TimePoint>& start_from) {
    std::lock_guard lock(this->simulator_mtx);
    auto simulators_it = this->simulators.find(robot);
    auto current_skill_data_it = this->current_skills.find(robot);
    if (simulators_it == this->simulators.end()) return;
    if (current_skill_data_it == this->current_skills.end()) return;

    if (wm == nullptr) {
        wm = this->real_wm;
    }
    auto simulator = std::make_shared<Simulator>(this->thread_pool, wm, this->am);
    simulator->setTasks({{robot, {std::get<0>(current_skill_data_it->second), td, {}}}});
    simulator->setResultCallback(result_callback);
    Simulator::startSimulation(simulator, this->should_run, std::get<2>(current_skill_data_it->second), start_from);
    simulators_it->second.push_back(simulator);
}

void SimulationManager::startSkillSimulation(const RobotIdentifier& robot, const Skill& skill, const TaskData& td,
                                             const ResultCallback& result_callback, const time::TimePoint& start_time,
                                             std::shared_ptr<const transform::WorldModel> wm) {
    if (wm == nullptr) {
        wm = this->real_wm;
    }
    auto simulator = std::make_shared<Simulator>(this->thread_pool, wm, this->am);
    simulator->setTasks({{robot, {&skill, td, {}}}});
    simulator->setResultCallback(result_callback);
    Simulator::startSimulation(simulator, this->should_run, start_time, std::nullopt);
    std::lock_guard lock(this->simulator_mtx);
    this->independent_simulators.push_back(simulator);
}

SimulationResult SimulationManager::runSyncSimulation(const RobotIdentifier& robot, const Skill& skill,
                                                      const TaskData& td, const time::TimePoint& start_time,
                                                      std::shared_ptr<const transform::WorldModel> wm) {
    if (wm == nullptr) {
        wm = this->real_wm;
    }
    auto simulator = std::make_shared<Simulator>(this->thread_pool, wm, this->am);
    simulator->setTasks({{robot, {&skill, td, {}}}});
    return simulator->runSyncSimulation(this->should_run, start_time);
}

// int SimulationManager::calculateScore(const SimulationResult& result) {
//     const time::Duration scanning_interval = 0.03;
//     std::optional<double> min_robot_distance;
//     // 1 - check for collision
//     for (const auto& task : result.tasks) {
//         // check min distance to other robot
//         for (time::TimePoint time = result.start_time; time < result.end_time; time += scanning_interval) {
//             std::vector<RobotIdentifier> visible_robots = result.wm->getVisibleRobots(time);

//             for (const auto& robot : visible_robots) {
//                 if (task.second.task_data.robot == robot) continue;
//                 auto robot_pose = result.wm->getTransform(task.second.task_data.robot.getFrame(), "", time);
//                 auto other_robot_pose = result.wm->getTransform(robot.getFrame(), "", time);
//                 if (robot_pose.has_value() && other_robot_pose.has_value()) {
//                     double distance =
//                         (robot_pose->transform.translation() - other_robot_pose->transform.translation()).norm();
//                     if (!min_robot_distance.has_value() || min_robot_distance.value() > distance)
//                         min_robot_distance = distance;
//                 }
//             }
//         }
//         if (min_robot_distance < localPlannerConfig().simulation_scoring_collision_distance.val()) {
//             return 0;
//         }
//     }
//     int score = 0;

//     // 3 - duration of path

//     double duration = time::Duration(result.end_time - result.start_time).asSec();

//     score += static_cast<int>(localPlannerConfig().simulation_scoring_duration_k / duration);

//     // 5 -  minimal distance to obstacle
//     if (min_robot_distance.has_value()) {
//         score += static_cast<int>(localPlannerConfig().simulation_scoring_obstacle_distance_k *
//                                   (1 - std::exp(localPlannerConfig().robot_radius * 2 -
//                                   min_robot_distance.value())));
//     } else {
//         score += static_cast<int>(localPlannerConfig().simulation_scoring_obstacle_distance_k);
//     }
//     return score;
// }

// std::vector<bool> SimulationManager::multiAgentCallback(
//     const std::vector<std::shared_ptr<const AbstractCFObstacle>>& features, const Eigen::Vector2d& goal_vec,
//     const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
//     time::TimePoint time) {
//     std::optional<std::map<unsigned long, bool>> rotation_vector_result =
//         this->getCookie<std::map<unsigned long, bool>>(td, "rotation_vectors");
//     if (!rotation_vector_result.has_value()) rotation_vector_result = std::map<unsigned long, bool>();
//     std::vector<bool> vectors;
//     vectors.reserve(features.size());
//     bool started_sim = false;
//     for (const auto& feature : features) {
//         auto rotation_vec_it = rotation_vector_result->find(feature->getUid());
//         if (rotation_vec_it != rotation_vector_result->end()) {
//             vectors.push_back(rotation_vec_it->second);
//         } else {
//             if (this->getCookie<bool>(td, "simulation").value_or(false) && !started_sim) {
//                 auto trans_to_feature = feature->getShape()->getTransformToClosestPoint(wm, td, robot, time);
//                 if (trans_to_feature.vec.has_value() && trans_to_feature.vec.value().norm() < 1.5) {
//                     TaskData simulation_task = td;
//                     auto other_rotation_vector = rotation_vector_result.value();
//                     rotation_vector_result.value()[feature->getUid()] = false;
//                     other_rotation_vector[feature->getUid()] = true;
//                     this->setCookie(td, "rotation_vectors", rotation_vector_result.value());
//                     this->setCookie(simulation_task, "rotation_vectors", other_rotation_vector);

//                     this->startNewSimulation(robot, wm, simulation_task, time);
//                     started_sim = true;
//                 }
//             }
//             vectors.push_back(false);
//         }
//     }
//     return vectors;
// }

}  // namespace luhsoccer::local_planner