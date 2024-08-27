#pragma once

#include "config_provider/config_base.hpp"

// Add your config file header here

namespace luhsoccer::config_provider {

struct DemoConfig;
struct ObserverConfig;
struct GameConfig;
struct SSLInterfaceConfig;
struct BallFilterConfig;
struct LuhvizConfig;
struct LuhvizInternalConfig;
struct RobotInterfaceConfig;
struct GameDataProviderConfig;
struct SkillsConfig;
struct SimulationInterfaceConfig;
struct StrategyConfig;
struct RobotControlConfig;
struct RobotControlVisualizationConfig;

/**
 * @brief A Struct which stores all Config objects
 *        The Config Objects are stored as a variable and in a list so that you can iterate over them
 */
struct ConfigStore : public ConfigStoreBase {
    ConfigStore();

    // Add your config file here (AS A REFERENCE)
    DemoConfig& demo_config;

    ObserverConfig& observer_config;
    GameConfig& game_config;

    SSLInterfaceConfig& ssl_interface_config;

    BallFilterConfig& ball_filter_config;
    LuhvizConfig& luhviz_config;
    LuhvizInternalConfig& luhviz_internal_config;
    RobotInterfaceConfig& robot_interface_config;

    GameDataProviderConfig& game_data_provider_config;
    SkillsConfig& skills_config;

    SimulationInterfaceConfig& simulation_interface_config;

    StrategyConfig& strategy_config;

    RobotControlConfig& robot_control_config;

    RobotControlVisualizationConfig& robot_control_visualization_config;
};

}  // namespace luhsoccer::config_provider
