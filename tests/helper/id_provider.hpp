#pragma once

#include "robot_identifier.hpp"

namespace luhsoccer::tests {
/**
 * @brief class that is allowed to create RobotIdentifiers for tests
 *
 */
class IDProvider {
   public:
    /**
     * @brief create a new RobotIdentifier
     *
     * @param id id of robot
     * @param team team
     * @return RobotIdentifier
     */
    static RobotIdentifier createID(int id, Team team) { return RobotIdentifier(id, team); }
};
}  // namespace luhsoccer::tests