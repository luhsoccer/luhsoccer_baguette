#pragma once

#include <optional>
#include "config/config_store.hpp"

namespace luhsoccer::config_provider {

class ConfigProvider {
   public:
    /**
     * @brief Used to get a reference to the ConfigStore
     *
     * @return ConfigStore& A reference to the ConfigStore
     */
    [[nodiscard]] static ConfigStore& getConfigStore() {
        if (!ConfigProvider::cs.has_value()) {
            ConfigProvider::cs.emplace(ConfigStore());
        }

        return *ConfigProvider::cs;
    }

   private:
    // ignore warnings on the next line so that we can have a globally available static object
    // which holds the ConfigStore
    // NOLINTNEXTLINE
    static inline std::optional<ConfigStore> cs = std::nullopt;
};

}  // namespace luhsoccer::config_provider