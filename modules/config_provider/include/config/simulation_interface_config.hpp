#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {

struct SimulationInterfaceConfig : public Config {
    SimulationInterfaceConfig() : Config("simulation_interface.toml") {}

    StringParam er_force_simulation_host =
        createStringParam("er_force_simulation_host",
                          "The host of the er_force simulation. Can also be an IP address. (Requires restart)",
                          "er_force_simulation", "127.0.0.1");

    IntParam er_force_vision_port =
        createIntParam("er_force_vision_port", "The vision port (TCP) of the er force simulation. (Requires restart)",
                       "network_connection", 10020, 0, 65535);

    IntParam er_force_simulation_control_port =
        createIntParam("er_force_simulation_control_port",
                       "The simulation control port (TCP) of the er force simulation. (Requires restart)",
                       "network_connection", 10300, 0, 65535);

    IntParam er_force_simulation_control_port_blue =
        createIntParam("er_force_simulation_control_port_blue",
                       "The blue control port (TCP) of the er force simulation. (Requires restart)",
                       "network_connection", 10301, 0, 65535);

    IntParam er_force_simulation_control_port_yellow =
        createIntParam("er_force_simulation_control_port_yellow",
                       "The yellow control port (TCP) of the er force simulation. (Requires restart)",
                       "network_connection", 10302, 0, 65535);

    StringParam simulator_geometry = createStringParam(
        "er_sim_simulator_geometry",
        "The simulator geometry to use. Valid values are: 2014, 2017, 2018, 2019, 2020, 2020B, 2023, 2023B",
        "er_sim_simulation", "2023");

    StringParam simulator_realism =
        createStringParam("er_sim_simulator_realism",
                          "The simulator realism to use. Valid values are: Friendly, None, RC2021 and Realistic",
                          "er_sim_simulation", "RC2021");
};

}  // namespace luhsoccer::config_provider