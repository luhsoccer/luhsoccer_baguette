#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {

struct RobotInterfaceConfig : public Config {
    RobotInterfaceConfig() : Config("robot_interface.toml") {}

    StringParam serial_port = createStringParam(
        "port", "The port to use for the serial connection. Set this to auto to automaticaly use a port",
        "serial_connection", "auto");

    StringParam network_hostname = createStringParam(
        "network_hostname", "The hostname of the base station. Can also be an IP address. (Requires restart)",
        "network_connection", "luhsoccer-bs");

    IntParam network_port = createIntParam("network_port", "The port of the base station. (Requires restart)",
                                           "network_connection", 46174, 0, 65535);
};

}  // namespace luhsoccer::config_provider