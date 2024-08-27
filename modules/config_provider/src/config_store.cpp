#include "config/config_store.hpp"

// Configs
#include "config/config_structs.hpp"
#include "config/observer_config.hpp"
#include "config/robot_interface_config.hpp"
#include "config/ball_filter_config.hpp"
#include "config/skills_config.hpp"
#include "config/game_config.hpp"
#include "config/luhviz_config.hpp"
#include "config/luhviz_internal_config.hpp"
#include "config/ssl_interface_config.hpp"
#include "config/simulation_interface_config.hpp"

#include "config/game_data_provider_config.hpp"

#include "config/strategy_config.hpp"

#include "config/robot_control_config.hpp"
#include "config/robot_control_visualization_config.hpp"

namespace luhsoccer::config_provider {
ConfigStore::ConfigStore()
    : ConfigStoreBase(),
      demo_config(addConfig<DemoConfig>()),
      observer_config(addConfig<ObserverConfig>()),
      game_config(addConfig<GameConfig>()),
      ssl_interface_config(addConfig<SSLInterfaceConfig>()),
      ball_filter_config(addConfig<BallFilterConfig>()),
      luhviz_config(addConfig<LuhvizConfig>()),
      luhviz_internal_config(addConfig<LuhvizInternalConfig>()),
      robot_interface_config(addConfig<RobotInterfaceConfig>()),
      game_data_provider_config(addConfig<GameDataProviderConfig>()),
      skills_config(addConfig<SkillsConfig>()),
      simulation_interface_config(addConfig<SimulationInterfaceConfig>()),
      strategy_config(addConfig<StrategyConfig>()),
      robot_control_config(addConfig<RobotControlConfig>()),
      robot_control_visualization_config(addConfig<RobotControlVisualizationConfig>()) {}

}  // namespace luhsoccer::config_provider
