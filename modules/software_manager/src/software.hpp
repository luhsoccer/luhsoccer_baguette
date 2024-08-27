#pragma once

#include <string_view>

namespace luhsoccer::software_manager::software {

namespace game_controller {
constexpr std::string_view VERSION = "3.9.0";

constexpr std::string_view DOWNLOAD_LINUX_AMD64 =
    "https://github.com/RoboCup-SSL/ssl-game-controller/releases/download/v3.9.0/"
    "ssl-game-controller_v3.9.0_linux_amd64";

constexpr std::string_view DOWNLOAD_WINDOWS_AMD64 =
    "https://github.com/RoboCup-SSL/ssl-game-controller/releases/download/v3.9.0/"
    "ssl-game-controller_v3.9.0_windows_amd64.exe";

}  // namespace game_controller

namespace simulation {
constexpr std::string_view VERSION = "v5";

constexpr std::string_view DOWNLOAD_LINUX_AMD64 =
    "https://gitlab.com/api/v4/projects/53771004/packages/generic/erforce-simulation/v5/simulator-cli-v5-linux-amd64";

constexpr std::string_view DOWNLOAD_WINDOWS_AMD64 =
    "https://gitlab.com/api/v4/projects/53771004/packages/generic/erforce-simulation/v5/"
    "simulator-cli-v5-windows-amd64.exe";

};  // namespace simulation
};  // namespace luhsoccer::software_manager::software