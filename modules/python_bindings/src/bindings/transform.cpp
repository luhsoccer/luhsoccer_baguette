#include "bindings.hpp"
#include "transform_helper/world_model_helper.hpp"

namespace luhsoccer::python {
using namespace transform;

template <>
void bindSharedClass(py::class_<WorldModel, std::shared_ptr<WorldModel>>& instance) {
    instance.def("getAllyRobotData", &WorldModel::getAllyRobotData, py::arg(), py::arg("time") = time::TimePoint(0));
    instance.def("getRobotData", &WorldModel::getRobotData, py::arg(), py::arg("time") = time::TimePoint(0));
    instance.def(
        "getTransform",
        [](const std::shared_ptr<WorldModel>& wm, const std::string& frame, const std::string& parent_frame,
           time::TimePoint when) {
            auto transform = wm->getTransform(frame, parent_frame, when);
            return transform;
        },
        py::arg(), py::arg("parent_frame") = std::string(""), py::arg("time") = time::TimePoint(0));
    instance.def("getGameState", &WorldModel::getGameState, py::arg("time") = time::TimePoint(0));
    instance.def(
        "getVisibleAllyRobots",
        [](const std::shared_ptr<WorldModel>& wm, time::TimePoint when) {
            auto robots = wm->getVisibleRobots<Team::ALLY>(when);
            std::vector<RobotHandle> handles;
            std::transform(robots.begin(), robots.end(), std::back_inserter(handles),
                           [&](const RobotIdentifier& id) { return RobotHandle(id, wm); });

            return handles;
        },
        py::arg("time") = time::TimePoint(0));
    instance.def(
        "getVisibleEnemyRobots",
        [](const std::shared_ptr<WorldModel>& wm, time::TimePoint when) {
            auto robots = wm->getVisibleRobots<Team::ENEMY>(when);
            std::vector<RobotHandle> handles;
            std::transform(robots.begin(), robots.end(), std::back_inserter(handles),
                           [&](const RobotIdentifier& id) { return RobotHandle(id, wm); });

            return handles;
        },
        py::arg("time") = time::TimePoint(0));

    instance.def("getPossibleAllyRobots", [](const std::shared_ptr<WorldModel>& wm) {
        auto robots = wm->getPossibleRobots<Team::ALLY>();
        std::vector<RobotHandle> handles;
        std::transform(robots.begin(), robots.end(), std::back_inserter(handles),
                       [&](const RobotIdentifier& id) { return RobotHandle(id, wm); });

        return handles;
    });
    instance.def("getPossibleEnemyRobots", [](const std::shared_ptr<WorldModel>& wm) {
        auto robots = wm->getPossibleRobots<Team::ENEMY>();
        std::vector<RobotHandle> handles;
        std::transform(robots.begin(), robots.end(), std::back_inserter(handles),
                       [&](const RobotIdentifier& id) { return RobotHandle(id, wm); });

        return handles;
    });
    instance.def("getBallPosition", &transform::helper::getBallPosition, py::arg("time") = time::TimePoint(0));
    instance.def("getBallVelocity", &transform::helper::getBallVelocity, py::arg("time") = time::TimePoint(0));
    instance.def("getBallInfo", &WorldModel::getBallInfo, py::arg("time") = time::TimePoint(0));
    instance.def(
        "getLastBallObtainPosition",
        [](const std::shared_ptr<WorldModel>& wm) -> std::optional<std::pair<time::TimePoint, Eigen::Vector2d>> {
            auto transform = wm->getLastBallObtainPosition();
            if (transform) {
                return std::make_pair(transform->first, transform->second.translation());
            } else {
                return std::nullopt;
            }
        });

    instance.def("createHandle",
                 [](const std::shared_ptr<WorldModel>& wm, const RobotIdentifier& id) { return RobotHandle(id, wm); });
}

template <>
void bindEnum(py::enum_<BallState>& instance) {
    instance.value("InRobot", BallState::IN_ROBOT);
    instance.value("Missing", BallState::MISSING);
    instance.value("OnField", BallState::ON_FIELD);
}

template <>
void bindClass(py::class_<BallInfo>& instance) {
    instance.def_readonly("getTime", &BallInfo::time);
    instance.def_readonly("getRobot", &BallInfo::robot);
    instance.def_readonly("getState", &BallInfo::state);
    instance.def("getPositionAndVelocity",
                 [](const BallInfo& info)
                     -> std::optional<std::pair<std::optional<Eigen::Vector2d>, std::optional<Eigen::Vector3d>>> {
                     auto value = info.position;

                     if (value) {
                         auto pos = value->first;
                         auto vel = value->second;

                         std::optional<Eigen::Vector2d> translation = std::nullopt;

                         if (pos) {
                             translation = pos->translation();
                         }

                         return std::make_optional(std::make_pair(translation, vel));
                     } else {
                         return std::nullopt;
                     }
                 });
}

template <>
void bindClass(py::class_<RobotHandle>& instance) {
    instance.def("getID", &RobotHandle::getID);
    instance.def("getPosition", &transform::helper::getPosition, py::arg("time") = time::TimePoint(0));
    instance.def("getVelocity", &transform::helper::getVelocity, py::arg("time") = time::TimePoint(0));
    instance.def("getPositionAndRotation", &transform::helper::getPositionAndRotation,
                 py::arg("time") = time::TimePoint(0));
    instance.def("__eq__", [](const RobotHandle& instance, const RobotHandle& other) {
        return instance.getID() == other.getID();  // @todo also compare WorldModel
    });
    instance.def("__hash__", [](const transform::RobotHandle& handle) {
        return std::hash<RobotIdentifier>{}(handle.getID());
    });  // @todo also hash WM

    instance.def("__lt__", [](const RobotHandle& instance, const RobotHandle& other) {
        return instance.getID() < other.getID();  // @todo also compare WorldModel
    });
}

template <>
void bindClass(py::class_<RobotData>& instance) {
    instance.def_readonly("on_field", &RobotData::on_field);
    instance.def_readonly("time", &RobotData::time);
}

template <>
void bindDerivedClass(py::class_<AllyRobotData, RobotData>& instance) {
    instance.def_readonly("ball_in_dribbler", &AllyRobotData::ball_in_dribbler);
    instance.def_readonly("cap_voltage", &AllyRobotData::cap_voltage);
}

template <>
void bindClass(py::class_<TransformHeader>& instance) {
    instance.def_readwrite("child_frame", &TransformHeader::child_frame);
    instance.def_readwrite("parent_frame", &TransformHeader::parent_frame);
    instance.def_readwrite("stamp", &TransformHeader::stamp);
}

template <>
void bindClass(py::class_<Transform>& instance) {
    instance.def_readwrite("header", &Transform::header);
    instance.def("getPositionAndRotation", [](const Transform& trans) -> Eigen::Vector3d {
        return {trans.transform.translation().x(), trans.transform.translation().y(),
                Eigen::Rotation2Dd(trans.transform.rotation()).angle()};
    });
    instance.def("setPositionAndRotation", [](Transform& trans, const Eigen::Vector3d& position_and_rotation) -> void {
        trans.transform = Eigen::Translation2d(position_and_rotation.x(), position_and_rotation.y()) *
                          Eigen::Rotation2Dd(position_and_rotation.z());
    });
}

template <>
void bindClass(py::class_<Position>& instance) {
    instance.def(py::init<std::string>());
    instance.def(py::init<std::string, double, double, double>(), py::arg(), py::arg("x") = 0.0, py::arg("y") = 0.0,
                 py::arg("t") = 0.0);
    instance.def("getFrame", &Position::getFrame);
}

template <>
void bindEnum(py::enum_<GameState>& instance) {
    instance.value("Halt", GameState::HALT);
    instance.value("Stop", GameState::STOP);
    instance.value("Normal", GameState::NORMAL);
    instance.value("BallPlacementFreeKick", GameState::BALL_PLACEMENT_FREE_KICK);
    instance.value("BallPlacementForceStart", GameState::BALL_PLACEMENT_FORCE_START);
    instance.value("BallPlacementEnemy", GameState::BALL_PLACEMENT_ENEMY);
    instance.value("KickoffPrep", GameState::KICKOFF_PREP);
    instance.value("KickoffPrepEnemy", GameState::KICKOFF_PREP_ENEMY);
    instance.value("Kickoff", GameState::KICKOFF);
    instance.value("KickoffEnemy", GameState::KICKOFF_ENEMY);
    instance.value("FreeKickPrep", GameState::FREE_KICK_PREP);
    instance.value("FreeKickPrepEnemy", GameState::FREE_KICK_PREP_ENEMY);
    instance.value("FreeKick", GameState::FREE_KICK);
    instance.value("KickEnemy", GameState::FREE_KICK_ENEMY);
    instance.value("PenaltyPrep", GameState::PENALTY_PREP);
    instance.value("PenaltyPrepEnemy", GameState::PENALTY_PREP_ENEMY);
    instance.value("Penalty", GameState::PENALTY);
    instance.value("PenaltyEnemy", GameState::PENALTY_ENEMY);
    instance.value("TimeoutEnemy", GameState::TIMEOUT_ENEMY);
    instance.value("TimeoutAlly", GameState::TIMEOUT_ALLY);
}

}  // namespace luhsoccer::python