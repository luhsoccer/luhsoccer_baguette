#include "simulation_manager.hpp"
#include "core/visit.hpp"
#include "event_system/event_system.hpp"
#include "simulator.hpp"
#include "color_util.hpp"

namespace luhsoccer::robot_control {

SimulationManager::SimulationManager(const std::shared_ptr<const transform::WorldModel>& global_wm,
                                     event_system::EventSystem& event_system)
    : event_system(event_system), global_wm(global_wm) {}

void SimulationManager::startSkillSimulation(const std::vector<SimulationTask>& tasks,
                                             const ResultCallback& result_callback, const time::TimePoint& start_time,
                                             std::shared_ptr<const transform::WorldModel> wm) {
    std::unique_lock lock(this->simulator_mtx);
    auto id = ++last_id;

    if (wm == nullptr) wm = this->global_wm;

    auto sim = std::make_shared<Simulator>(id, wm, tasks, start_time);
    auto job = [sim, result_callback, id, this]() {
        result_callback(id, sim->runSimulation());
        std::unique_lock lock(this->simulator_mtx);
        this->simulators.erase(id);
    };
    this->simulators[id] = sim;

    this->event_system.run(job);
};

SimulationResult SimulationManager::runSyncSimulation(const std::vector<SimulationTask>& tasks,
                                                      const time::TimePoint& start_time,
                                                      std::shared_ptr<const transform::WorldModel> wm) {
    std::unique_lock lock(this->simulator_mtx);
    auto id = ++last_id;
    if (wm == nullptr) wm = this->global_wm;
    auto sim = std::make_shared<Simulator>(id, wm, tasks, start_time);
    this->simulators[id] = sim;
    lock.unlock();

    auto result = sim->runSimulation();

    lock.lock();
    this->simulators.erase(id);
    return result;
}

void SimulationManager::stopSimulations() {
    std::unique_lock lock(this->simulator_mtx);
    for (auto& [id, sim] : this->simulators) {
        sim->stopSimulation();
    }
    this->simulators.clear();
}

void SimulationManager::stopSimulation(unsigned long id) {
    std::unique_lock lock(this->simulator_mtx);
    auto sim_it = this->simulators.find(id);
    if (sim_it != this->simulators.end()) {
        sim_it->second->stopSimulation();
        this->simulators.erase(sim_it);
    }
}

void visualizeSimulationResult(unsigned long id, const SimulationResult& result, const MarkerAdapter& ma,
                               const SimulationVizOptions& options) {
    for (const auto& [robot, task] : result.tasks) {
        visualizeSimulationTask(id, result, robot, ma, options);
    }
}

marker::Color mapVelToColor(double velocity, const SimulationVizOptions::MapVelocity& options) {
    HSV fast_hsv = rgb2hsv({options.fast_color.red, options.fast_color.green, options.fast_color.blue});
    HSV slow_hsv = rgb2hsv({options.slow_color.red, options.slow_color.green, options.slow_color.blue});

    velocity = std::max(options.min_velocity, std::min(options.max_velocity, velocity));

    double ratio = (velocity - options.min_velocity) / (options.max_velocity - options.min_velocity);

    RGB color_rgb =
        hsv2rgb({fast_hsv.h * ratio + slow_hsv.h * (1 - ratio), fast_hsv.s * ratio + slow_hsv.s * (1 - ratio),
                 fast_hsv.v * ratio + slow_hsv.v * (1 - ratio)});
    marker::Color color(color_rgb.r, color_rgb.g, color_rgb.b);

    return color;
}

void visualizeSimulationTask(unsigned long /*id*/, const SimulationResult& result, const RobotIdentifier& robot,
                             const MarkerAdapter& ma, const SimulationVizOptions& options) {
    std::vector<marker::Point> points;
    std::vector<marker::Color> colors;
    points.reserve(static_cast<int>(time::Duration(result.end_time - result.start_time).asSec() / options.sampling));
    transform::Position robot_pos(robot.getFrame());
    for (time::TimePoint t = result.start_time; t < result.end_time; t += time::Duration(options.sampling)) {
        auto pos = robot_pos.getCurrentPosition(result.wm, "", t);
        auto vel = robot_pos.getVelocity(result.wm, "", "", t);

        if (!pos.has_value() || !vel.has_value()) continue;

        points.emplace_back(pos->translation().x(), pos->translation().y());

        auto color_map_visitor = overload{[&colors, &vel](const SimulationVizOptions::MapVelocity& map_velocity) {
                                              colors.push_back(mapVelToColor(vel->head<2>().norm(), map_velocity));
                                          },
                                          [](const marker::Color&) {}};
        std::visit(color_map_visitor, options.color);
    }

    marker::LineStrip line_strip({""}, options.ns.value_or(fmt::format("{}Simulation", robot)), options.id);
    line_strip.setPoints(points);
    line_strip.setLifetime(options.lifetime);
    line_strip.setThickness(options.thickness);

    auto color_visitor =
        overload{[&colors, &line_strip](const SimulationVizOptions::MapVelocity&) { line_strip.setColors(colors); },
                 [&line_strip](const marker::Color& color) { line_strip.setColor(color); }};
    std::visit(color_visitor, options.color);

    if (points.size() < 2) return;
    ma.displayMarker(line_strip);

    if (options.show_time) {
        // get position at mid of trajectory
        auto position = points[points.size() / 2];
        marker::Text text({"", position.x, position.y}, options.ns.value_or(fmt::format("{}SimulationTime", robot)),
                          options.id + 1);
        text.setText(fmt::format("{:.2f}s", time::Duration(result.end_time - result.start_time).asSec()));
        text.setColor(options.time_color);
        text.setScale(options.time_scaling);
        ma.displayMarker(text);
    }
}

}  // namespace luhsoccer::robot_control