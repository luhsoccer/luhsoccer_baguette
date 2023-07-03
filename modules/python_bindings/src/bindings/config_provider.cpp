#include "bindings.hpp"

namespace luhsoccer::python {

template <class T>
void addConfigParams(py::class_<T>& instance, config_provider::Config& config) {
    for (const auto& [param_name, param_ptr] : config.getParams()) {
        switch (param_ptr->getType()) {
            case luhsoccer::config_provider::datatypes::Type::INT: {
                auto& el = dynamic_cast<luhsoccer::config_provider::IntParam>(*param_ptr);
                instance.def(param_name.c_str(), [&el]() { return el.val(); });
                break;
            }
            case luhsoccer::config_provider::datatypes::Type::DOUBLE: {
                auto& el = dynamic_cast<luhsoccer::config_provider::DoubleParam>(*param_ptr);
                instance.def(param_name.c_str(), [&el]() { return el.val(); });
                break;
            }
            case luhsoccer::config_provider::datatypes::Type::BOOL: {
                auto& el = dynamic_cast<luhsoccer::config_provider::BoolParam>(*param_ptr);
                instance.def(param_name.c_str(), [&el]() { return el.val(); });
                break;
            }
            case luhsoccer::config_provider::datatypes::Type::STRING: {
                auto& el = dynamic_cast<luhsoccer::config_provider::StringParam>(*param_ptr);
                instance.def(param_name.c_str(), [&el]() { return el.val(); });
                break;
            }
        }
    }
}

template <class T>
void addConfig(py::module_& baguette_module, config_provider::Config& config) {
    py::class_<T> instance(baguette_module, config.getConfigName().c_str());
    addConfigParams(instance, config);
}

void bindConfigs(py::module_& baguette_module) {
    const auto& cs = config_provider::ConfigProvider::getConfigStore();

    // ADD CONFIGS HERE
    addConfig<config_provider::DemoConfig>(baguette_module, cs.demo_config);
    addConfig<config_provider::ObserverConfig>(baguette_module, cs.observer_config);
    addConfig<config_provider::GameConfig>(baguette_module, cs.game_config);
}

}  // namespace luhsoccer::python