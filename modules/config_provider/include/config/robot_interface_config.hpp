#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {

struct RobotInterfaceConfig : public Config {
    RobotInterfaceConfig() : Config("robot_interface.toml") {}

    StringParam network_hostname = createStringParam(
        "network_hostname", "The hostname of the base station. Can also be an IP address. (Requires restart)",
        "network_connection", "luhsoccer-bs.local");
    StringParam secondary_network_hostname =
        createStringParam("secondary_network_hostname",
                          "The hostname of the second base station. Only used if is is also found (optional)",
                          "network_connection", "luhsoccer-b2.local");

    BoolParam use_secondary_bs = createBoolParam("use_secondary_bs", "Whether to also use the secondary base station",
                                                 "network_connection", false);

    IntParam network_port = createIntParam("network_port", "The port of the base station. (Requires restart)",
                                           "network_connection", 46174, 0, 65535);

    StringParam first_bs_robots =
        createStringParam("first_bs_robots",
                          "All the robots that should be adressed by the first basestation. "
                          "All other robots will be adressed by the second basestation (REQUIRES RESTART)"
                          "Make sure that the frequencies on the robots are set correctly",
                          "network_connection", "0, 2, 4, 6, 8, 10, 12, 14");
};

}  // namespace luhsoccer::config_provider