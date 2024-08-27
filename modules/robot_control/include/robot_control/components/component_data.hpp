#pragma once
#include <utility>

#include "robot_control/skills/task_data.hpp"

namespace luhsoccer::robot_control {
class CooperationModule;
class MarkerAdapter;
struct ComponentData {
    ComponentData(std::shared_ptr<const transform::WorldModel> wm, const TaskData& td, const RobotIdentifier& robot,
                  CooperationModule& coop, const time::TimePoint& time, const MarkerAdapter& ma)
        : wm(std::move(wm)), td(td), robot(robot), coop(coop), time(time), ma(ma){};

    ComponentData(const ComponentData& data, const MarkerAdapter& other_ma)
        : wm(data.wm), td(data.td), robot(data.robot), coop(data.coop), time(data.time), ma(other_ma){};
    std::shared_ptr<const transform::WorldModel> wm;
    const TaskData& td;
    RobotIdentifier robot;
    CooperationModule& coop;
    time::TimePoint time{0};
    const MarkerAdapter& ma;
};
}  // namespace luhsoccer::robot_control