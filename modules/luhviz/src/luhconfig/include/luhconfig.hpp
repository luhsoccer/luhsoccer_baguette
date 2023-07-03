#pragma once

#include "imgui.h"
#include "config/config_store.hpp"
#include "config_provider/config_store_main.hpp"
#include "imgui_backend/imgui_stdlib.h"
#include "include/data_proxy.hpp"
#include "new_rendering/include/gl_texture.hpp"

#include <deque>

namespace luhsoccer::luhviz {
class LuhvizConfig {
   public:
    LuhvizConfig(DataProxy& proxy) : data_proxy(proxy) {}
    void init();
    void render(bool* open);

    /**
     * @brief saves all config values on button click
     *
     */
    void saveConfig();

    /**
     * @brief updates the values in config_provider and saves only the luhviz settings permanently
     *
     */
    void saveAtClose();

   private:
    luhsoccer::logger::Logger logger{"luhviz/luhconfig"};
    DataProxy& data_proxy;

    std::chrono::system_clock::time_point last_write_time{std::chrono::system_clock::now()};
    std::chrono::system_clock::time_point last_update_time{std::chrono::system_clock::now()};
    constexpr static long WRITE_VALUES_DELAY_MS{500L};    // 0.5 sec delay
    constexpr static long UPDATE_VALUES_DELAY_MS{5000L};  // 5 sec delay

    std::deque<bool> bools{};
    std::vector<float> floats{};
    std::vector<int> ints{};
    std::vector<std::string> strings;

    const std::string reset_icon_path = "res/images/reset_icon.png";
    GLTexture reset_icon;

    bool saved = true;
};
}  // namespace luhsoccer::luhviz