#include "include/window_layout_handler.hpp"
#include "config/luhviz_internal_config.hpp"

namespace luhsoccer::luhviz {
void WindowLayoutHandler::loadLayout() {
    this->render_view_open = config_provider::ConfigProvider::getConfigStore().luhviz_internal_config.render_view_open;
    this->inspector_open = config_provider::ConfigProvider::getConfigStore().luhviz_internal_config.inspector_open;
    this->console_open = config_provider::ConfigProvider::getConfigStore().luhviz_internal_config.console_open;
    this->manipulator_open = config_provider::ConfigProvider::getConfigStore().luhviz_internal_config.manipulator_open;
    this->game_info_open = config_provider::ConfigProvider::getConfigStore().luhviz_internal_config.game_info_open;
    this->software_manager_open =
        config_provider::ConfigProvider::getConfigStore().luhviz_internal_config.software_manager_open;
    this->game_log_open = config_provider::ConfigProvider::getConfigStore().luhviz_internal_config.game_log_open;
    this->robert_display_open =
        config_provider::ConfigProvider::getConfigStore().luhviz_internal_config.robert_display_open;
    this->info_display_open =
        config_provider::ConfigProvider::getConfigStore().luhviz_internal_config.info_display_open;
    this->plotter_open = config_provider::ConfigProvider::getConfigStore().luhviz_internal_config.plotter_open;
    this->render_view_fullscreen = config_provider::ConfigProvider::getConfigStore().luhviz_internal_config.fullscreen;
}

void WindowLayoutHandler::setDefaultLayout() {
    // TODO: also update luhviz.ini file from luhviz_default.ini
    this->render_view_open = true;
    this->inspector_open = true;
    this->console_open = true;
    this->manipulator_open = true;
    this->game_info_open = true;
    this->game_log_open = true;
    this->robert_display_open = true;
    this->info_display_open = true;
    this->plotter_open = true;
    this->software_manager_open = true;
    this->render_view_fullscreen = false;
}
}  // namespace luhsoccer::luhviz