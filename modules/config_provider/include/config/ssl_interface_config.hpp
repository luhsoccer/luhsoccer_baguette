#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {
struct SSLInterfaceConfig : public Config {
    SSLInterfaceConfig() : Config("ssl_interface.toml") {}

    StringParam vision_ip =
        createStringParam("vision_ip", "The ip of the vision (requires restart)", "ssl_connection", "224.5.23.2");

    IntParam vision_port =
        createIntParam("vision_port", "The port of the vision (requires restart)", "ssl_connection", 10006);

    StringParam gc_ip =
        createStringParam("gc_ip", "The ip of the gc (requires restart)", "gc_connection", "224.5.23.1");

    IntParam gc_port = createIntParam("gc_port", "The port of the gc (requires restart)", "serial_connection", 10003);

    IntParam ignore_camera = createIntParam(
        "ignore_camera", "The id of the camera whose vision packets should be thrown away (-1 to ignore no camera)",
        "general", -1);

    IntParam ignore_side =
        createIntParam("ignore_side", "Which side of camera data to ignore (+1=positive halve; -1=negative halve)",
                       "vision_data_filtering", 0, -1, 1);
};

}  // namespace luhsoccer::config_provider