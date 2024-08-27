#pragma once

#include <glm/glm.hpp>
#include <string>
#include <utility>
#include "config_provider/parameters.hpp"
#include "core/robot_identifier.hpp"
#include "game_data_provider/team_info.hpp"
#include "transform/game_state.hpp"

namespace luhsoccer::luhviz {
struct RobotData {
    size_t id{};
    std::string ns{};
    glm::dvec2 position{};
    double rotation{};
};

struct BallData {
    std::string ns{};
    glm::dvec2 position{};
};

enum class ManipulationMode {
    SELECT,
    TELEPORT_ROBOT,
    TELEPORT_BALL,
    ADD_POINT,
    ADD_DIRECTION,
    EXECUTE_SKILL,
    BALL_SLINGSHOT,
    MEASURE
};

struct GameDataDisplay {
    std::optional<game_data_provider::TeamInfo> ally_info{};
    std::optional<game_data_provider::TeamInfo> enemy_info{};
    std::optional<transform::GameState> game_state{};
};

enum class ParamType { NONE, INT, DOUBLE, BOOL, STRING };

class ConfigParam {
   public:
    virtual ~ConfigParam() = default;
    ConfigParam(const ConfigParam&) = default;
    ConfigParam(ConfigParam&&) = delete;
    ConfigParam& operator=(const ConfigParam&) = default;
    ConfigParam& operator=(ConfigParam&&) = delete;

    ConfigParam() = default;
    ConfigParam(std::string config, std::string group, std::string key, std::string description, bool is_writeable)
        : config(std::move(config)),
          group(std::move(group)),
          key(std::move(key)),
          description(std::move(description)),
          is_writeable(is_writeable) {}

    virtual ParamType getType() { return ParamType::NONE; };
    virtual void reset() {}
    virtual bool hasValueChanged() { return false; }

    std::string config{"default"};
    std::string group{"Global"};
    std::string key{""};
    std::string description{""};
    bool is_writeable{true};
    bool synced_with_config_provider{true};
};

class ConfigInt : public ConfigParam {
   public:
    ConfigInt(int v, std::string config, std::string group, std::string key, std::string description, bool is_writeable,
              int default_value, int min = std::numeric_limits<int>::min(), int max = std::numeric_limits<int>::max())
        : ConfigParam(std::move(config), std::move(group), std::move(key), std::move(description), is_writeable),
          val(v),
          min(min),
          max(max),
          default_value(default_value) {}
    int val;
    int min;
    int max;
    int default_value;

    ParamType getType() override { return ParamType::INT; }
    void reset() override { val = default_value; }
    bool hasValueChanged() override { return default_value != val; }
    [[nodiscard]] int getDefaultValue() const { return default_value; }
};

class ConfigDouble : public ConfigParam {
   public:
    ConfigDouble(double v, std::string config, std::string group, std::string key, std::string description,
                 bool is_writeable, double default_value, double min = std::numeric_limits<double>::min(),
                 double max = std::numeric_limits<double>::max())
        : ConfigParam(std::move(config), std::move(group), std::move(key), std::move(description), is_writeable),
          val(v),
          min(min),
          max(max),
          default_value(default_value) {}
    double val;
    double min;
    double max;
    double default_value;

    ParamType getType() override { return ParamType::DOUBLE; }
    void reset() override { val = default_value; }
    bool hasValueChanged() override {
        constexpr int MULT = 1000;
        // compare only 3 decimals
        return std::round(default_value * MULT) != std::round(val * MULT);
    }
    [[nodiscard]] double getDefaultValue() const { return default_value; }
};

class ConfigBool : public ConfigParam {
   public:
    ConfigBool(bool v, std::string config, std::string group, std::string key, std::string description,
               bool is_writeable, bool default_value)
        : ConfigParam(std::move(config), std::move(group), std::move(key), std::move(description), is_writeable),
          val(v),
          default_value(default_value) {}
    bool val;
    bool default_value;

    ParamType getType() override { return ParamType::BOOL; }
    void reset() override { val = default_value; }
    bool hasValueChanged() override { return default_value != val; }
    [[nodiscard]] bool getDefaultValue() const { return default_value; }
};

class ConfigString : public ConfigParam {
   public:
    ConfigString(std::string v, std::string config, std::string group, std::string key, std::string description,
                 bool is_writeable, std::string default_value)
        : ConfigParam(std::move(config), std::move(group), std::move(key), std::move(description), is_writeable),
          val(std::move(v)),
          default_value(std::move(default_value)) {}
    std::string val;
    std::string default_value;

    ParamType getType() override { return ParamType::STRING; }
    void reset() override { val = default_value; }
    bool hasValueChanged() override { return default_value.compare(val) != 0; }
    [[nodiscard]] std::string getDefaultValue() const { return default_value; }
};
}  // namespace luhsoccer::luhviz