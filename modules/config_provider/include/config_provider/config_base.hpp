#pragma once

#include <iostream>
#include <memory>
#include <limits>
#include <utility>
#include <algorithm>

#include "common_types.hpp"
#include "utils/utils.hpp"
#include "parameters.hpp"
#include "utility.hpp"
#include "logger/logger.hpp"

namespace luhsoccer::config_provider {

const std::string CMAKERC_PATH = "configs";

/**
 * @brief A base Class for all config structs
 * Provides functions used to create Parameters and load and save to/from files
 */
class Config {
   public:
    /**
     * @brief Construct a new Config object
     *
     * @param filename The name of the .toml file the config should be saved in
     * @param config_name The name of the config (as shown in luhviz) (leave empty to derive it from filename)
     */
    Config(const std::string& filename, datatypes::ConfigType type = datatypes::ConfigType::LOCAL)
        : config_type(type), pure_filename(filename) {
        this->config_name = util::deriveConfigName(filename);

        // convert config name to upper case
        std::string uppercase_filename(this->config_name.size(), ' ');
        std::transform(this->config_name.begin(), this->config_name.end(), uppercase_filename.begin(), ::toupper);

        // check if an environment variable has been set for this config
        const char* prefix = std::getenv((std::string("BAGUETTE_CONFIG_PREFIX_") + uppercase_filename).c_str());

        if (prefix != nullptr) {
            LOG_WARNING(logger::Logger("config_provider"),
                        "Using config prefix {} for config {}. You can ignore this warning if it is intentional",
                        prefix, this->pure_filename);
            this->pure_filename = std::string(prefix) + "_" + this->pure_filename;

            // derive config name again so that it includes the prefix
            this->config_name = util::deriveConfigName(this->pure_filename);
        }

        const auto home_path = getBaguetteDirectory();

        if (home_path.empty()) {
            LOG_WARNING(logger::Logger("config_provider"), "No Baguette Directory Found!");
        } else {
            auto config_path = home_path;

#ifdef BAGUETTE_LOCAL_MODE
            // if we are in DEV-MODE go into the cmake_rc folder if the config is a shared-config
            // Otherwise we already are in the runtime folder
            if (this->config_type == datatypes::ConfigType::SHARED) {
                config_path = config_path.parent_path();
                config_path /= "extern/cmake_rc";
            }
#endif

            // now we either are in the "HOME/-luhsoccer_baguette", "[...]/luhsoccer_baguette/runtime" or
            // "[...]/luhsoccer_baguette/extern/cmake_rc" directory go into the config directory
            // So now go into the config director in each one of them
            config_path /= "configs";

            // check if the config directory exists. If not create it
            if (!std::filesystem::exists(config_path)) {
                std::filesystem::create_directory(config_path);
            }

            // create the path to this config file
            this->config_home_filepath = (config_path / this->pure_filename).string();
        }

        // assign a default value to the table
        this->table = toml::table();

        // try to load the content of the config from memory
        this->updateTable();
    }

    template <typename ParamType>
    ParamType& createParam(std::unique_ptr<ParamType> param_ptr) {
        const auto& key = param_ptr->getKey();

        param_ptr->load();

        const std::string_view dbg_key_store = param_ptr->getKey();

        auto result = this->params.insert({param_ptr->getKey(), std::move(param_ptr)});

        if (!result.second) {
            LOG_ERROR(logger::Logger("config_provider"), "Parameter name '{}' repeated! Change keys to be unique!",
                      dbg_key_store);
        }

        // For the case that no config file currently exists, create one
        // this->save();

        return dynamic_cast<ParamType&>(*this->params[key]);
    }

    void save() {
        for (const auto& el_pair : params) {
            el_pair.second->updateTable();
        }

        if (this->config_home_filepath.empty()) {
            return;
        }

        std::lock_guard<std::mutex> lock(this->toml_table_mutex);

#ifdef BAGUETTE_LOCAL_MODE
        // if we are in DEV-MODE and the cofig is a shared config, overwrite the config in the cmake_rc memory
        // completely
        // if this is a local config, save the config in the project-runtime folder
        // ([...]/luhsoccer_baguette/runtime)
        if (config_type == datatypes::ConfigType::SHARED) {
            // create a toml table which contains the newest version of all parameters
            toml::table new_table = util::concatTables(this->table, this->diff_table);
            util::saveFile(this->config_home_filepath, new_table);
        } else {
            util::saveFile(this->config_home_filepath, this->diff_table);
        }

#else
        // if we are in local mode (DEPLOY-MODE) save the table differences to the HOME-directory
        util::saveFile(this->config_home_filepath, this->diff_table);
#endif
    }

    void load() {
        this->updateTable();

        // load each variable
        for (const auto& el_pair : params) {
            el_pair.second->load();
        }
    }

    BoolParamClass& createBoolParam(const std::string& key, const std::string& description,
                                    const std::string& group = "", bool value = false, bool writable = true) {
        auto param = std::make_unique<BoolParamClass>(this->table, this->diff_table, key, description, group, value,
                                                      writable, this->toml_table_mutex);
        return this->createParam(std::move(param));
    }

    IntParamClass& createIntParam(const std::string& key, const std::string& description, const std::string& group = "",
                                  int value = 0, int min = std::numeric_limits<int>::min(),
                                  int max = std::numeric_limits<int>::max(), bool writable = true) {
        auto param = std::make_unique<IntParamClass>(this->table, this->diff_table, key, description, group, value, min,
                                                     max, writable, this->toml_table_mutex);
        return this->createParam(std::move(param));
    }

    DoubleParamClass& createDoubleParam(const std::string& key, const std::string& description,
                                        const std::string& group = "", double value = 0.0f,
                                        double min = std::numeric_limits<double>::lowest(),
                                        double max = std::numeric_limits<double>::max(), bool writable = true) {
        auto param = std::make_unique<DoubleParamClass>(this->table, this->diff_table, key, description, group, value,
                                                        min, max, writable, this->toml_table_mutex);
        return this->createParam(std::move(param));
    }

    StringParamClass& createStringParam(const std::string& key, const std::string& description,
                                        const std::string& group = "", const std::string& value = "",
                                        bool writable = true) {
        auto param = std::make_unique<StringParamClass>(this->table, this->diff_table, key, description, group, value,
                                                        writable, this->toml_table_mutex);
        return this->createParam(std::move(param));
    }

    /**
     * @brief Get the map containing all Parameters of this config
     *
     * @return const std::unordered_map<std::string, std::unique_ptr<Param>>& A map containing all parameters of this
     * config
     */
    const std::unordered_map<std::string, std::unique_ptr<Param>>& getParams() { return params; }

    /**
     * @brief Get the Name of the config
     *
     * @return const std::string& The Name of the COnfig
     */
    const std::string& getConfigName() const { return this->config_name; }

    /**
     * @brief Resets all changed parameter-values
     *
     */
    void resetValues() {
        for (const auto& [_, p] : this->getParams()) {
            p->resetValue();
        }
    }

   private:
    /**
     * @brief Loads the internal toml table from memory
     */
    void updateTable() {
        std::lock_guard<std::mutex> lock(this->toml_table_mutex);

        // first read the file from cmakerc memory and then overwrite it with local changes from toml files in the
        // globally defined config folder

        const std::string cmake_rc_file_path = CMAKERC_PATH + "/" + this->pure_filename;
        if (!util::parseFile(cmake_rc_file_path, this->table, true)) {
            LOG_DEBUG(logger::Logger("config_provider"), "File '{}' could not be opened!", cmake_rc_file_path);
        }

        if (!this->config_home_filepath.empty()) {
            if (!util::parseFile(this->config_home_filepath, this->diff_table, false)) {
                LOG_DEBUG(logger::Logger("config_provider"), "Optional (local) File '{}' could not be opened!",
                          this->config_home_filepath);
            }
        }
    }

   private:
    std::unordered_map<std::string, std::unique_ptr<Param>> params;
    mutable std::mutex toml_table_mutex;
    toml::table table;
    toml::table diff_table;
    datatypes::ConfigType config_type;
    std::string config_home_filepath = "";
    std::string pure_filename;
    std::string config_name = "";
};

class ConfigStoreBase {
   public:
    /**
     * @brief Get the List of Config objects
     *
     * @return const std::vector<std::unique_ptr<Config>>& The List of Config Objects
     */
    [[nodiscard]] const std::vector<std::unique_ptr<Config>>& getConfigs() { return this->config_list; }

    void saveAll() {
        for (const auto& cfg : this->getConfigs()) {
            cfg->save();
        }
    }

    void loadAll() {
        for (const auto& cfg : this->getConfigs()) {
            cfg->load();
        }
    }

   private:
    /**
     * @brief Stores a list of Config Objects (Used to iterate over all Lists)
     */
    std::vector<std::unique_ptr<Config>> config_list;

   protected:
    /**
     * @brief A Function used to create a shared pointer to a Config Struct, return it, and add it to the list
     *
     * @tparam T The Config Struct type
     * @return std::uniqure_ptr<T> A Shared pointer to a created Config Object of type T
     */
    template <typename T>
    T& addConfig() {
        auto cfg_ptr = std::make_unique<T>();
        config_list.emplace_back(std::move(cfg_ptr));
        return static_cast<T&>(*config_list.back());
    }
};

}  // namespace luhsoccer::config_provider
