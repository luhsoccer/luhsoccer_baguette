#pragma once
#include <limits>
#include "common_types.hpp"

namespace luhsoccer {

namespace transform {
struct RobotDataStorage;
}
namespace game_data_provider {  // Needed because the game data provider gets the data from the visions and can create
class GameDataProvider;         // ids
}
namespace robot_interface {  // Needed because the robot interface must read the internal id to send the packet to the
class RobotInterface;        // corresponding robot
}

namespace simulation_interface {  // Needed because the simulation does some conversions between ids and numbers
class SimulationInterface;
}
namespace luhviz {  // Needed because the simulation does some conversions between ids and numbers
class RenderView;
class SkillTester;
class RobotController;
class RobertDisplay;
class DataProxy;
class LuhvizMain;  // temp
}  // namespace luhviz
namespace marker {
class RobotInfo;  // needed for robot identification
}
namespace tests {
class IDProvider;
}
namespace python {
class IDProvider;
}

namespace scenario {
class Scenario;
}

namespace robot_control {
class RobotControlModule;
}
class RobotIdentifier {
   protected:
    constexpr RobotIdentifier(size_t id, Team team) noexcept : id(id), team(team){};

    size_t id;
    Team team;

   public:
    [[nodiscard]] constexpr bool isAlly() const noexcept { return this->team == Team::ALLY; }
    [[nodiscard]] constexpr bool isEnemy() const noexcept { return this->team == Team::ENEMY; }
    [[nodiscard]] constexpr Team getTeam() const noexcept { return this->team; }
    [[nodiscard]] std::string getFrame() const;

    friend constexpr bool operator==(const RobotIdentifier& lhs, const RobotIdentifier& rhs) {
        return lhs.id == rhs.id && lhs.team == rhs.team;
    }

    friend constexpr bool operator!=(const RobotIdentifier& lhs, const RobotIdentifier& rhs) { return !(lhs == rhs); }

    friend constexpr bool operator<(const RobotIdentifier& lhs, const RobotIdentifier& rhs) {
        if (lhs.team != rhs.team) {
            return false;
        }
        return lhs.id < rhs.id;
    }

    friend std::string format_as(RobotIdentifier handle);

    friend struct std::hash<RobotIdentifier>;
    friend struct transform::RobotDataStorage;
    friend class game_data_provider::GameDataProvider;
    friend class robot_interface::RobotInterface;
    friend class simulation_interface::SimulationInterface;
    friend class luhviz::RenderView;
    friend class luhviz::SkillTester;
    friend class luhviz::RobotController;
    friend class luhviz::RobertDisplay;
    friend class luhviz::DataProxy;
    friend class luhviz::LuhvizMain;  // temp
    friend class marker::RobotInfo;
    friend class scenario::Scenario;
    friend class tests::IDProvider;
    friend class python::IDProvider;
    friend class robot_control::RobotControlModule;

    constexpr static RobotIdentifier create_empty() {
        return RobotIdentifier{std::numeric_limits<size_t>().max(), Team::ALLY};
    }
};

inline constexpr RobotIdentifier EMPTY_IDENTIFIER = RobotIdentifier::create_empty();

}  // namespace luhsoccer
template <>
struct std::hash<luhsoccer::RobotIdentifier> {
    std::size_t operator()(const luhsoccer::RobotIdentifier& identifier) const noexcept {
        const size_t h1 = std::hash<size_t>{}(identifier.id);
        const size_t h2 = std::hash<luhsoccer::Team>{}(identifier.team);

        return h1 ^ (h2 << 1);
    };
};