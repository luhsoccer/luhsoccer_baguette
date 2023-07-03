#include "bindings.hpp"

#include "observer/continuous_observer.hpp"
#include "observer/static_observer.hpp"
#include "observer/static_observer_position.hpp"

namespace luhsoccer::python {

class StaticObserver {};
class StaticObserverPosition {};

using namespace game_data_provider;
using namespace observer;

template <>
void bindModule(py::module_& baguette_module, py::class_<GameDataProvider>& instance) {
    loadEnumBindings<StrategyType>(baguette_module, "StrategyType");
    loadClassBindings<AllyRobot::BestPassReceiver>(baguette_module, "BestPassReceiver");
    loadClassBindings<BallHolder>(baguette_module, "BallHolder");
    loadSharedClassBindings<Observer>(baguette_module, "Observer");

    loadClassBindings<StaticObserver>(baguette_module, "StaticObserver");
    loadClassBindings<StaticObserverPosition>(baguette_module, "StaticObserverPosition");

    loadClassBindings<TeamInfo>(baguette_module, "TeamInfo");

    instance.def("getWorldModel", &GameDataProvider::getWorldModel, py::return_value_policy::copy);
    instance.def("getObserver", &GameDataProvider::getObserver, py::return_value_policy::reference);
    instance.def("getSpecialKickPosition", &GameDataProvider::getSpecialKickPosition);
    instance.def("getSpecialKickTime", &GameDataProvider::getSpecialKickTime);
    instance.def("getGoalie", [](const GameDataProvider& gdp) {
        return transform::RobotHandle(gdp.getGoalie(), gdp.getWorldModel());
    });
    instance.def("getEnemyGoalie", [](const GameDataProvider& gdp) {
        return transform::RobotHandle(gdp.getEnemyGoalie(), gdp.getWorldModel());
    });

    instance.def("getAllyTeamInfo", &GameDataProvider::getAllyTeamInfo);
    instance.def("getEnemyTeamInfo", &GameDataProvider::getEnemyTeamInfo);
}

template <>
void bindEnum(py::enum_<StrategyType>& instance) {
    instance.value("Defensive", StrategyType::DEFENSIVE);
    instance.value("Offensive", StrategyType::OFFENSIVE);
}

template <>
void bindClass(py::class_<AllyRobot::BestPassReceiver>& instance) {
    instance.def_readonly("handle", &AllyRobot::BestPassReceiver::handle);
    instance.def_readonly("score", &AllyRobot::BestPassReceiver::score);
}

template <>
void bindClass(py::class_<BallHolder>& instance) {
    instance.def_readonly("handle", &BallHolder::handle);
    instance.def_readonly("ball_obtained_pos", &BallHolder::ball_obtained_pos);
    instance.def_readonly("movement_allowed", &BallHolder::movement_allowed);
}

template <>
void bindSharedClass(py::class_<Observer, std::shared_ptr<Observer>>& instance) {
    instance.def("getBallCarrier", &Observer::getBallCarrier);
    instance.def("getBallControllingTeam", &Observer::getBallControllingTeam);
    instance.def("getBallGoalProbability", &Observer::getBallGoalProbability);
    instance.def("getBallObtainedPos", &Observer::getBallObtainedPos);
    instance.def("getGoalProbability", &Observer::getGoalProbability);
    instance.def("getThreatLevel", &Observer::getThreatLevel);
    instance.def("getBestPassReceiver", &Observer::getBestPassReceiver);
    instance.def("getStrategyType", &Observer::getStrategyType);
    instance.def("getLastBallTouchingRobot", &Observer::getLastBallTouchingRobot);
    instance.def("getBallTouchingForbiddenRobot", &Observer::getBallTouchingForbiddenRobot);
}

/**
 * @brief All Static Observer Methods which take a RobotHandle or a WorldModel
 *
 */
template <>
void bindClass(py::class_<StaticObserver>& instance) {
    namespace calc = observer::calculation;

    instance.def_static("calculateBallPosession", &calc::calculateBallPosession, py::arg("gdp"));

    instance.def_static(
        "calculateGoalProbability",
        py::overload_cast<const transform::RobotHandle&, const time::TimePoint>(&calc::calculateGoalProbability),
        py::arg("handle"), py::arg("time") = time::TimePoint(0));

    instance.def_static(
        "calculateShootPoint",
        py::overload_cast<const transform::RobotHandle&, const time::TimePoint>(&calc::calculateShootPoint),
        py::arg("handle"), py::arg("time") = time::TimePoint(0));

    instance.def_static("calculateThreatLevel",
                        py::overload_cast<const transform::RobotHandle&>(&calc::calculateThreatLevel),
                        py::arg("handle"));

    instance.def_static(
        "calculateBestPassReceiver",
        py::overload_cast<const transform::RobotHandle&, std::vector<RobotIdentifier>, const time::TimePoint>(
            &calc::calculateBestPassReceiver),
        py::arg("handle"), py::arg("allies_to_evaluate") = std::vector<RobotIdentifier>{},
        py::arg("time") = time::TimePoint(0));

    instance.def_static("calculatePassProbability",
                        py::overload_cast<const transform::RobotHandle&, const RobotIdentifier&,
                                          const std::shared_ptr<const transform::WorldModel>&, const time::TimePoint>(
                            &calc::calculatePassProbability),
                        py::arg("passer"), py::arg("receiver"), py::arg("world_model"),
                        py::arg("time") = time::TimePoint(0));

    instance.def_static(
        "isPassLineDefended",
        py::overload_cast<const transform::RobotHandle&, const transform::RobotHandle&, const time::TimePoint>(
            &calc::isPassLineDefended),
        py::arg("handle"), py::arg("other_handle"), py::arg("time") = time::TimePoint(0));

    instance.def_static(
        "isOutgoingPassDefended",
        py::overload_cast<const transform::RobotHandle&, const time::TimePoint>(&calc::isOutgoingPassDefended),
        py::arg("handle"), py::arg("time") = time::TimePoint(0));

    instance.def_static(
        "calculateBestGoalPoint",
        py::overload_cast<const transform::RobotHandle&, std::optional<RobotIdentifier>, const time::TimePoint>(
            &calc::calculateBestGoalPoint),
        py::arg("handle"), py::arg("id_to_ignorer") = std::nullopt, py::arg("time") = time::TimePoint(0));

    instance.def_static("calculateMostLikelyShootPoint",
                        py::overload_cast<const transform::RobotHandle&, const game_data_provider::GameDataProvider&,
                                          const time::TimePoint>(&calculation::calculateMostLikelyShootPoint),
                        py::arg("enemy"), py::arg("gdp"), py::arg("time") = time::TimePoint(0));

    instance.def_static("calculateFutureBallPos", &calc::calculateFutureBallPos, py::arg("world_model"),
                        py::arg("time") = time::TimePoint(0));

    instance.def_static("calculateInterceptionRobotIsViable",
                        py::overload_cast<const transform::RobotHandle&, const time::TimePoint>(
                            &calculation::calculateInterceptionRobotIsViable),
                        py::arg("handle"), py::arg("time") = time::TimePoint(0));

    instance.def_static(
        "calculateBestInterceptor",
        py::overload_cast<const transform::WorldModel&, std::vector<RobotIdentifier>, const time::TimePoint>(
            &calculation::calculateBestInterceptor),
        py::arg("world_model"), py::arg("viable_interceptors") = std::vector<RobotIdentifier>{},
        py::arg("time") = time::TimePoint(0));
}

/**
 * @brief All Static Observer Methods which take a transform::Position
 *
 */
template <>
void bindClass(py::class_<StaticObserverPosition>& instance) {
    namespace calc = observer::calculation;

    instance.def_static(
        "calculateGoalProbability",
        py::overload_cast<const transform::Position&, Team, std::shared_ptr<const transform::WorldModel>,
                          const time::TimePoint>(&calc::calculateGoalProbability),
        py::arg("pos"), py::arg("team"), py::arg("world_model"), py::arg("time") = time::TimePoint(0));

    instance.def_static(
        "calculateBestGoalPoint",
        py::overload_cast<const transform::Position&, Team, std::shared_ptr<const transform::WorldModel>,
                          std::optional<RobotIdentifier>, const time::TimePoint>(&calc::calculateBestGoalPoint),
        py::arg("pos"), py::arg("team"), py::arg("world_model"), py::arg("id_to_ignore") = std::nullopt,
        py::arg("time") = time::TimePoint(0));

    instance.def_static("calculateThreatLevel",
                        py::overload_cast<const transform::Position&, std::shared_ptr<const transform::WorldModel>,
                                          const time::TimePoint>(&calc::calculateThreatLevel),
                        py::arg("pos"), py::arg("world_model"), py::arg("time") = time::TimePoint(0));

    instance.def_static("isPassLineDefended",
                        py::overload_cast<const transform::Position&, const transform::Position&, const Team,
                                          std::shared_ptr<const transform::WorldModel>, const time::TimePoint>(
                            &calc::isPassLineDefended),
                        py::arg("pos"), py::arg("other_pos"), py::arg("team"), py::arg("world_model"),
                        py::arg("time") = time::TimePoint(0));

    instance.def_static(
        "isOutgoingPassDefended",
        py::overload_cast<const transform::Position&, Team, std::shared_ptr<const transform::WorldModel>,
                          std::optional<RobotIdentifier>, const time::TimePoint>(&calc::isOutgoingPassDefended),
        py::arg("robot_pos"), py::arg("team"), py::arg("world_model"), py::arg("robot_id") = std::nullopt,
        py::arg("time") = time::TimePoint(0));

    instance.def_static(
        "calculateBestPassReceiver",
        py::overload_cast<const transform::Position&, Team, std::shared_ptr<const transform::WorldModel>,
                          std::vector<RobotIdentifier>, std::optional<RobotIdentifier>, const time::TimePoint>(
            &calc::calculateBestPassReceiver),
        py::arg("pos"), py::arg("team"), py::arg("world_model"),
        py::arg("allies_to_evaluate") = std::vector<RobotIdentifier>{}, py::arg("passing_robot") = std::nullopt,
        py::arg("time") = time::TimePoint(0));

    instance.def_static("calculatePassProbability",
                        py::overload_cast<const transform::Position&, const transform::Position&, const Team,
                                          const std::shared_ptr<const transform::WorldModel>&, const time::TimePoint>(
                            &calc::calculatePassProbability),
                        py::arg("passer_pos"), py::arg("receiver_pos"), py::arg("team"), py::arg("world_model"),
                        py::arg("time") = time::TimePoint(0));

    instance.def_static(
        "calculateShootPoint",
        py::overload_cast<const transform::Position&, Team, std::shared_ptr<const transform::WorldModel>,
                          const time::TimePoint>(&calc::calculateShootPoint),
        py::arg("pos"), py::arg("team"), py::arg("world_model"), py::arg("time") = time::TimePoint(0));

    instance.def_static("calculatePassSuccessChance", &calculation::calcPassSuccessChance, py::arg("start_pos"),
                        py::arg("end_pos"), py::arg("kicker_pos"), py::arg("enemies"), py::arg("time"),
                        py::arg("pass_power"), py::arg("enemy_max_speed"), py::arg("radius_scaling"),
                        py::arg("pass_power_lut"), py::arg("turn_time_lut"), py::arg("pass_time_lut"));
}

template <>
void bindClass(py::class_<TeamInfo>& instance) {
    instance.def_readonly("ball_placement_failure", &TeamInfo::ball_placement_failures);
    instance.def_readonly("ball_placement_failures_reached", &TeamInfo::ball_placement_failures_reached);
    instance.def_readonly("can_place_ball", &TeamInfo::can_place_ball);
    instance.def_readonly("bot_substitution_intent", &TeamInfo::bot_substitution_intent);
    instance.def_readonly("foul_counter", &TeamInfo::foul_counter);
    instance.def_readonly("goalkeeper", &TeamInfo::goalkeeper);
    instance.def_readonly("max_allowed_bots", &TeamInfo::max_allowed_bots);
    instance.def_readonly("name", &TeamInfo::name);
    instance.def_readonly("red_cards", &TeamInfo::red_cards);
    instance.def_readonly("yellow_cards", &TeamInfo::yellow_cards);
    instance.def_readonly("score", &TeamInfo::score);
    instance.def_readonly("yellow_card_times", &TeamInfo::yellow_card_times);
    instance.def_readonly("timeout_time", &TeamInfo::timeout_time);
    instance.def_readonly("timeouts", &TeamInfo::timeouts);
}

}  // namespace luhsoccer::python