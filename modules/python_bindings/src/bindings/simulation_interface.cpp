#include "bindings.hpp"
#include "config/game_config.hpp"

namespace luhsoccer::python {

using namespace simulation_interface;

template <>
void bindModule(nb::module_& baguette_module, nb::class_<SimulationInterface>& instance) {
    loadEnumBindings<SimulationConnectorType>(baguette_module, "SimulationConnectorType");
    instance.def("getConnector", &SimulationInterface::getConnector);
    instance.def("switchConnector", &SimulationInterface::switchConnector);
    instance.def(
        "teleportBall",
        [](SimulationInterface& self, const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) {
            // Wrapper since Affine2d is currently not supported in python
            Eigen::Affine2d affine = Eigen::Translation2d(position.x(), position.y()) * Eigen::Rotation2Dd(0.0);
            if (config_provider::ConfigProvider::getConfigStore().game_config.is_flipped) {
                affine.translation().x() *= -1.0;
                affine.translation().y() *= -1.0;
            }
            self.teleportBall(affine, velocity);
        },
        nb::arg("position"), nb::arg("velocity") = Eigen::Vector3d{0.0, 0.0, 0.0});
    instance.def("kickBall", &SimulationInterface::kickBall, nb::arg("velocity"));
    instance.def(
        "teleportRobot",
        [](SimulationInterface& self, const Eigen::Vector3d& position_and_rotation, const RobotIdentifier& which,
           const Eigen::Vector3d& velocity, bool present = true) {
            // @TODO change later if team colors are available
            TeamColor color = TeamColor::BLUE;

            if (!config_provider::ConfigProvider::getConfigStore().game_config.is_blue) {
                color = getOppositeTeamColor(color);
            }

            if (which.getTeam() != Team::ALLY) {
                color = getOppositeTeamColor(color);
            }

            auto position_rotation_copy = position_and_rotation;

            if (config_provider::ConfigProvider::getConfigStore().game_config.is_flipped) {
                position_rotation_copy.x() *= -1.0;
                position_rotation_copy.y() *= -1.0;
                position_rotation_copy.z() += L_PI;
            }

            Eigen::Affine2d affine = Eigen::Translation2d(position_rotation_copy.x(), position_rotation_copy.y()) *
                                     Eigen::Rotation2Dd(position_rotation_copy.z());

            self.teleportRobot(IDProvider::getNumericID(which), color, affine, velocity, present);
        },
        nb::arg("position_and_rotation"), nb::arg("robot_id"), nb::arg("velocity") = Eigen::Vector3d{0.0, 0.0, 0.0},
        nb::arg("present") = true);
}

}  // namespace luhsoccer::python