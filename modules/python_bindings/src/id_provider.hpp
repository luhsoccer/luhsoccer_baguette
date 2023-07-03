#pragma once

#include "robot_identifier.hpp"

namespace luhsoccer::python {

class IDProvider {
   public:
    static unsigned int getNumericID(const RobotIdentifier& id) { return id.id; }
};

}  // namespace luhsoccer::python