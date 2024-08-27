#include "bindings.hpp"
#include "config/config_structs.hpp"
#include "config/game_config.hpp"
#include "config/observer_config.hpp"
#include "config/strategy_config.hpp"
#include "config/skills_config.hpp"

namespace luhsoccer::python {

template <class T>
void addConfigParams(nb::class_<T>& instance, config_provider::Config& config) {
    for (const auto& [param_name, param_ptr] : config.getParams()) {
        switch (param_ptr->getType()) {
            case luhsoccer::config_provider::datatypes::Type::INT: {
                auto& el = dynamic_cast<luhsoccer::config_provider::IntParam>(*param_ptr);
                instance.def_static(param_name.c_str(), [&el]() { return el.val(); });
                break;
            }
            case luhsoccer::config_provider::datatypes::Type::DOUBLE: {
                auto& el = dynamic_cast<luhsoccer::config_provider::DoubleParam>(*param_ptr);
                instance.def_static(param_name.c_str(), [&el]() { return el.val(); });
                break;
            }
            case luhsoccer::config_provider::datatypes::Type::BOOL: {
                auto& el = dynamic_cast<luhsoccer::config_provider::BoolParam>(*param_ptr);
                instance.def_static(param_name.c_str(), [&el]() { return el.val(); });
                break;
            }
            case luhsoccer::config_provider::datatypes::Type::STRING: {
                auto& el = dynamic_cast<luhsoccer::config_provider::StringParam>(*param_ptr);
                instance.def_static(param_name.c_str(), [&el]() { return el.val(); });
                break;
            }
        }
    }
}

template <class T>
void addConfig(nb::module_& baguette_module, config_provider::Config& config) {
    nb::class_<T> instance(baguette_module, config.getConfigName().c_str());
    baguette_module.def(
        fmt::format("config_{}", config.getConfigName().c_str()).c_str(),
        [&config]() -> T& { return static_cast<T&>(config); }, nb::rv_policy::reference);
    addConfigParams(instance, config);
}

void bindConfigs(nb::module_& baguette_module) {
    const auto& cs = config_provider::ConfigProvider::getConfigStore();

    // ADD CONFIGS HERE
    addConfig<config_provider::DemoConfig>(baguette_module, cs.demo_config);
    addConfig<config_provider::ObserverConfig>(baguette_module, cs.observer_config);
    addConfig<config_provider::GameConfig>(baguette_module, cs.game_config);
    addConfig<config_provider::StrategyConfig>(baguette_module, cs.strategy_config);
    addConfig<config_provider::SkillsConfig>(baguette_module, cs.skills_config);
}

}  // namespace luhsoccer::python