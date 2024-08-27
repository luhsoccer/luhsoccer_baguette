#include "simulation_interface/simulation_interface.hpp"

#include "simulations/test_simulation_connector.hpp"
#include "simulations/erforce_simulation_connector.hpp"
#include "simulations/er_sim_connector.hpp"

#include "luhsoccer_simulation_control.pb.h"
#include "vision_processor/vision_processor_events.hpp"

namespace luhsoccer::simulation_interface {

void SimulationInterface::switchConnector(const SimulationConnectorType type) {
    std::lock_guard lock(this->connector_mutex);
    if (type != this->active_connector) {
        logger.info("Switch from {} to {}", this->active_connector, type);
        if (this->active_connector != simulation::SimulationConnectorType::NONE) {
            this->connectors[this->active_connector]->stop();
        }
        this->active_connector = type;
        if (this->active_connector != simulation::SimulationConnectorType::NONE) {
            this->connectors[this->active_connector]->load();
        }
    }
}

template <typename T, typename... Args>
void SimulationInterface::addConnector(Args&&... args) {
    std::unique_ptr<T> simulation =
        std::make_unique<T>(event_system, packet_sink, robot_sink, simulation_sink, std::forward<Args>(args)...);
    logger.debug("Added simulation connector: {}", simulation->type());
    connectors[simulation->type()] = std::move(simulation);
}

void SimulationInterface::setup() {
    addConnector<TestSimulationConnector>();
    addConnector<ErforceSimulationConnector>();
    addConnector<ErSimConnector>();
}

SimulationConnectorType SimulationInterface::getConnector() const {
    std::lock_guard lock(this->connector_mutex);
    return this->active_connector;
}

void SimulationInterface::stop() {
    std::unique_lock lock(this->connector_mutex);
    if (this->active_connector != simulation::SimulationConnectorType::NONE) {
        this->connectors[this->active_connector]->stop();
    }
}

void SimulationInterface::sendRobotCommand(const RobotControl& control, const TeamColor color) {
    std::lock_guard lock(this->connector_mutex);
    if (this->active_connector != simulation::SimulationConnectorType::NONE) {
        this->connectors[active_connector]->onRobotCommand(control, color);
    }
}
void SimulationInterface::sendSimulationCommand(const LuhsoccerSimulatorControl& control) {
    std::lock_guard lock(this->connector_mutex);
    if (this->active_connector != simulation::SimulationConnectorType::NONE) {
        this->connectors[active_connector]->onSimulationCommand(control);
    }
}

void SimulationInterface::teleportRobot(unsigned int which, TeamColor team, const Eigen::Affine2d& target,
                                        const Eigen::Vector3d& velocity, bool present) {
    LuhsoccerSimulatorControl control{};
    auto teleport = control.mutable_control()->add_teleport_robot();
    teleport->mutable_id()->set_id(which);
    switch (team) {
        case TeamColor::BLUE:
            teleport->mutable_id()->set_team(::Team::BLUE);
            break;
        case TeamColor::YELLOW:
            teleport->mutable_id()->set_team(::Team::YELLOW);
            break;
        default:
            break;
    }
    const auto pos = target.translation();
    teleport->set_x(static_cast<float>(pos.x()));
    teleport->set_y(static_cast<float>(pos.y()));
    const auto rot = Eigen::Rotation2Dd(target.rotation()).angle();
    teleport->set_orientation(static_cast<float>(rot));

    teleport->set_v_x(static_cast<float>(velocity.x()));
    teleport->set_v_y(static_cast<float>(velocity.y()));
    teleport->set_v_angular(static_cast<float>(velocity.z()));

    teleport->set_present(present);

    this->sendSimulationCommand(control);

    // event
    vision_processor::TeleportData event_data;
    event_data.teleport_type = vision_processor::TeleportData::RobotTeleport(which, team);
    event_data.new_pose = target;
    event_data.new_velocity = velocity;
    event_data.time = time::now();
    event_system.fireEvent(vision_processor::TeleportEvent(event_data));
}

void SimulationInterface::teleportBall(const Eigen::Affine2d& target, const Eigen::Vector3d& velocity) {
    LuhsoccerSimulatorControl control{};
    auto ball_teleport = control.mutable_control()->mutable_teleport_ball();
    const auto& pos = target.translation();
    ball_teleport->set_x(static_cast<float>(pos.x()));
    ball_teleport->set_y(static_cast<float>(pos.y()));
    ball_teleport->set_z(0.0);

    ball_teleport->set_vx(static_cast<float>(velocity.x()));
    ball_teleport->set_vy(static_cast<float>(velocity.y()));
    ball_teleport->set_vz(static_cast<float>(velocity.z()));

    this->sendSimulationCommand(control);

    // event
    vision_processor::TeleportData event_data;
    event_data.teleport_type = vision_processor::TeleportData::BallTeleport{};
    event_data.new_pose = target;
    event_data.new_velocity = velocity;
    event_data.time = time::now();
    event_system.fireEvent(vision_processor::TeleportEvent(event_data));
}

void SimulationInterface::kickBall(const Eigen::Vector3d& velocity) {
    LuhsoccerSimulatorControl control{};
    auto ball_kick = control.mutable_control()->mutable_teleport_ball();
    ball_kick->set_vx(static_cast<float>(velocity.x()));
    ball_kick->set_vy(static_cast<float>(velocity.y()));
    ball_kick->set_vz(static_cast<float>(velocity.z()));

    this->sendSimulationCommand(control);
}

}  // namespace luhsoccer::simulation_interface