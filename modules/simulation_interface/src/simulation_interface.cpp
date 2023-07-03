#include "simulation_interface/simulation_interface.hpp"

#include "simulations/test_simulation_connector.hpp"
#include "simulations/erforce_simulation_connector.hpp"
#include "luhsoccer_simulation_control.pb.h"

namespace luhsoccer::simulation_interface {

void SimulationInterface::switchConnector(const SimulationConnectorType type) {
    std::lock_guard lock(this->connector_mutex);
    if (type != this->active_connector) {
        LOG_INFO(logger, "Switch from {} to {}", this->active_connector, type);
        if (this->active_connector != simulation::SimulationConnectorType::NONE) {
            this->connectors[this->active_connector]->stop();
        }
        this->active_connector = type;
        if (this->active_connector != simulation::SimulationConnectorType::NONE) {
            this->connectors[this->active_connector]->load();
        }
    }
}

void SimulationInterface::setup() {
    addConnector<TestSimulationConnector>();
    addConnector<ErforceSimulationConnector>();
}

SimulationConnectorType SimulationInterface::getConnector() const {
    std::lock_guard lock(this->connector_mutex);
    return this->active_connector;
}

void SimulationInterface::loop(std::atomic_bool& should_run) {
    while (should_run) {
        std::unique_lock lock(this->connector_mutex);

        if (this->active_connector != simulation::SimulationConnectorType::NONE) {
            auto& connector = this->connectors[this->active_connector];
            auto& rate = connector->getRate();
            connector->update();
            lock.unlock();
            rate.sleep();
        } else {
            std::this_thread::yield();
        }
    }
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
    ball_teleport->set_vz(0.0);

    this->sendSimulationCommand(control);
}

}  // namespace luhsoccer::simulation_interface