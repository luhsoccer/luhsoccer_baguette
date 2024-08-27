#include "bindings.hpp"
#include "transform_helper/world_model_helper.hpp"

namespace luhsoccer::python {
using namespace transform;

template <>
void bindClass(nb::class_<WorldModel>& instance) {
    instance.def("getAllyRobotData", &WorldModel::getAllyRobotData, nb::arg(), nb::arg("time") = time::TimePoint(0));
    instance.def("getRobotData", &WorldModel::getRobotData, nb::arg(), nb::arg("time") = time::TimePoint(0));
    instance.def(
        "getTransform",
        [](const std::shared_ptr<WorldModel>& wm, const std::string& frame, const std::string& parent_frame,
           time::TimePoint when) {
            auto transform = wm->getTransform(frame, parent_frame, when);
            return transform;
        },
        nb::arg(), nb::arg("parent_frame") = std::string(""), nb::arg("time") = time::TimePoint(0));
    instance.def("getGameState", &WorldModel::getGameState, nb::arg("time") = time::TimePoint(0));
    instance.def(
        "getVisibleAllyRobots",
        [](const std::shared_ptr<WorldModel>& wm, time::TimePoint when) {
            auto robots = wm->getVisibleRobots<Team::ALLY>(when);
            std::vector<RobotHandle> handles;
            std::transform(robots.begin(), robots.end(), std::back_inserter(handles),
                           [&](const RobotIdentifier& id) { return RobotHandle(id, wm); });

            return handles;
        },
        nb::arg("time") = time::TimePoint(0));
    instance.def(
        "getVisibleEnemyRobots",
        [](const std::shared_ptr<WorldModel>& wm, time::TimePoint when) {
            auto robots = wm->getVisibleRobots<Team::ENEMY>(when);
            std::vector<RobotHandle> handles;
            std::transform(robots.begin(), robots.end(), std::back_inserter(handles),
                           [&](const RobotIdentifier& id) { return RobotHandle(id, wm); });

            return handles;
        },
        nb::arg("time") = time::TimePoint(0));

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

    instance.def("getBallPositionOr", &transform::WorldModel::getBallPositionOr, nb::arg("default_value"));
    instance.def("getBallPosition", &transform::WorldModel::getBallPosition);

    instance.def("getBallVelocityOr", &transform::WorldModel::getBallVelocityOr, nb::arg("default_value"));
    instance.def("getBallVelocity", &transform::WorldModel::getBallVelocity);

    instance.def("getBallInfo", &WorldModel::getBallInfo, nb::arg("time") = time::TimePoint(0));
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
void bindClass(nb::class_<BallInfo>& instance) {
    instance.def_ro("getTime", &BallInfo::time);
    instance.def_ro("getRobot", &BallInfo::robot);
    instance.def_ro("getState", &BallInfo::state);
    instance.def("getPositionAndVelocity",
                 [](const BallInfo& info) -> std::optional<std::pair<Eigen::Vector2d, Eigen::Vector3d>> {
                     auto value = info.position;

                     if (value) {
                         auto pos = value->first;
                         auto vel = value->second;
                         return std::make_optional(std::make_pair(pos.translation(), vel));
                     } else {
                         return std::nullopt;
                     }
                 });
}

template <>
void bindClass(nb::class_<RobotHandle>& instance) {
    instance.def("getID", &RobotHandle::getID);
    instance.def("isWorldModelValid", &RobotHandle::isWorldModelValid);

    instance.def("isAlly", &RobotHandle::isAlly);
    instance.def("isEnemy", &RobotHandle::isEnemy);
    instance.def("getTeam", &RobotHandle::getTeam);
    instance.def("getFrame", &RobotHandle::getFrame);

    instance.def("getPositionOr", &transform::RobotHandle::getPosVecOr, nb::arg("default_value"));
    instance.def("getPosition", &transform::RobotHandle::getPosVec);

    instance.def("getVelocityOr", &transform::RobotHandle::getVelocityOr, nb::arg("default_value"));
    instance.def("getVelocity", &transform::RobotHandle::getVelocity);

    instance.def("getPositionAndRotationOr", &transform::RobotHandle::getPosAndRotVecOr, nb::arg("default_value"));
    instance.def("getPositionAndRotation", &transform::RobotHandle::getPosAndRotVec);

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
void bindClass(nb::class_<RobotData>& instance) {
    instance.def_ro("on_field", &RobotData::on_field);
    instance.def_ro("time", &RobotData::time);
}

template <>
void bindDerivedClass(nb::class_<AllyRobotData, RobotData>& instance) {
    instance.def_ro("ball_in_dribbler", &AllyRobotData::ball_in_dribbler);
    instance.def_ro("cap_voltage", &AllyRobotData::cap_voltage);
}

template <>
void bindClass(nb::class_<TransformHeader>& instance) {
    instance.def_rw("child_frame", &TransformHeader::child_frame);
    instance.def_rw("parent_frame", &TransformHeader::parent_frame);
    instance.def_rw("stamp", &TransformHeader::stamp);
}

template <>
void bindClass(nb::class_<Transform>& instance) {
    instance.def_rw("header", &Transform::header);
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
void bindClass(nb::class_<Position>& instance) {
    instance.def(nb::init<std::string>());
    instance.def(nb::init<std::string, double, double, double>(), nb::arg(), nb::arg("x") = 0.0, nb::arg("y") = 0.0,
                 nb::arg("t") = 0.0);
    instance.def("getFrame", &Position::getFrame);
    instance.def(
        "getCurrentPositionAndRotation",
        [](const Position& self, const std::shared_ptr<WorldModel>& world_model, const Position& reference,
           const time::TimePoint& time) -> std::optional<Eigen::Vector3d> {
            std::optional<Eigen::Affine2d> current_pos = self.getCurrentPosition(world_model, reference, time);
            if (current_pos) {
                return Eigen::Vector3d(current_pos->translation().x(), current_pos->translation().y(),
                                       Eigen::Rotation2Dd(current_pos->rotation()).angle());
            } else {
                return std::nullopt;
            }
        },
        "world_model"_a, "reference"_a = Position(""), "time"_a = time::TimePoint(0));
    instance.def(
        "getCurrentPositionAndRotation",
        [](const Position& self, const std::shared_ptr<WorldModel>& world_model,
           const Eigen::Vector3d& default_position, Position& reference,
           const time::TimePoint& time) -> Eigen::Vector3d {
            std::optional<Eigen::Affine2d> current_pos = self.getCurrentPosition(world_model, reference, time);
            if (current_pos) {
                return {current_pos->translation().x(), current_pos->translation().y(),
                        Eigen::Rotation2Dd(current_pos->rotation()).angle()};
            } else {
                return default_position;
            }
        },
        "world_model"_a, "default_position"_a, "reference"_a = Position(""), "time"_a = time::TimePoint(0));
    instance.def("__repr__", &Position::getString);
}

template <>
void bindClass(nb::class_<FieldData>& instance) {
    instance.def_ro("division", &FieldData::division);
    instance.def_ro("size", &FieldData::size);
    instance.def_ro("goal_width", &FieldData::goal_width);
    instance.def_ro("goal_depth", &FieldData::goal_depth);
    instance.def_ro("boundary_width", &FieldData::boundary_width);
    instance.def_ro("penalty_area_depth", &FieldData::penalty_area_depth);
    instance.def_ro("penalty_area_width", &FieldData::penalty_area_width);
    instance.def_ro("center_circle_radius", &FieldData::center_circle_radius);
    instance.def_ro("line_thickness", &FieldData::line_thickness);
    instance.def_ro("goal_center_to_penalty_mark", &FieldData::goal_center_to_penalty_mark);
    instance.def_ro("goal_height", &FieldData::goal_height);
    instance.def_ro("ball_radius", &FieldData::ball_radius);
    instance.def_ro("max_robot_radius", &FieldData::max_robot_radius);
    instance.def_ro("field_runoff_width", &FieldData::field_runoff_width);
}

}  // namespace luhsoccer::python