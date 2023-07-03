#pragma once

#include "config_provider/config_base.hpp"

// Add your config file header here
#include "config/config_structs.hpp"
#include "config/observer_config.hpp"
#include "config/local_planner_components_config.hpp"
#include "config/robot_interface_config.hpp"
#include "config/ball_filter_config.hpp"
#include "config/skills_config.hpp"
#include "config/game_config.hpp"
#include "config/local_planner_visualization_config.hpp"
#include "config/luhviz_config.hpp"
#include "config/luhviz_internal_config.hpp"
#include "config/ssl_interface_config.hpp"

#include "config/local_planner_components_config.hpp"
#include "config/game_data_provider_config.hpp"
namespace luhsoccer::config_provider {

/**
 * @brief A Struct which stores all Config objects
 *        The Config Objects are stored as a variable and in a list so that you can iterate over them
 */
struct ConfigStore : public ConfigStoreBase {
    // Add your config file here (AS A REFERENCE)
    DemoConfig& demo_config = addConfig<DemoConfig>();

    ObserverConfig& observer_config = addConfig<ObserverConfig>();
    GameConfig& game_config = addConfig<GameConfig>();

    SSLInterfaceConfig& ssl_interface_config = addConfig<SSLInterfaceConfig>();

    LocalPlannerComponentsConfig& local_planner_components_config = addConfig<LocalPlannerComponentsConfig>();
    BallFilterConfig& ball_filter_config = addConfig<BallFilterConfig>();
    LocalPlannerVisualizationConfig& local_planner_visualization_config = addConfig<LocalPlannerVisualizationConfig>();
    LuhvizConfig& luhviz_config = addConfig<LuhvizConfig>();
    LuhvizInternalConfig& luhviz_internal_config = addConfig<LuhvizInternalConfig>();
    RobotInterfaceConfig& robot_interface_config = addConfig<RobotInterfaceConfig>();

    GameDataProviderConfig& game_data_provider_config = addConfig<GameDataProviderConfig>();
    SkillsConfig& skills_config = addConfig<SkillsConfig>();
};

}  // namespace luhsoccer::config_provider
