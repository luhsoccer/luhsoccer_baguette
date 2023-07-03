#pragma once

#include "imgui.h"
#include "include/data_proxy.hpp"
#include "robot_identifier.hpp"
#include "new_rendering/include/gl_texture.hpp"

namespace luhsoccer::luhviz {
class RobotController {
   public:
    RobotController(DataProxy& proxy) : proxy(proxy) {}

    void init();
    void render(bool* open, const std::vector<size_t>& active_controllers);
    bool isGlobalSteering();

   private:
    luhsoccer::logger::Logger logger{"luhviz/robot_controller"};

    DataProxy& proxy;
    std::vector<RobotIdentifier> robot_ids{};
    std::string ids_as_string{""};
    bool global_movement = true;

    std::vector<size_t> registered_controllers{};

    void displayControllRobotUI(const std::vector<size_t>& active_controllers, size_t i);

    const std::string gamepad_icon_path = "res/images/controller_icons/gamepad.png";
    GLTexture gamepad_icon;

    [[nodiscard]] const std::string stringFromIds(const std::vector<RobotIdentifier>& robot_ids) const {
        using namespace std::string_literals;
        std::string result = "-\0"s;
        for (const auto& id : robot_ids) {
            result += std::to_string(id.id) + "\0"s;
        }
        result += "\0"s;
        return result;
    }

    /**
     * @brief runs to all active controllers to register newly added and removes absent
     *
     * @param active_controllers
     */
    void manageControllerRegistration(const std::vector<size_t>& active_controllers);
};
}  // namespace luhsoccer::luhviz