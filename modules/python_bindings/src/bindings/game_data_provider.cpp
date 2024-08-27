#include "bindings.hpp"

#include "observer/continuous_observer.hpp"
#include "observer/static_observer.hpp"
#include "observer/static_observer_position.hpp"
#include "observer/observer_events.hpp"

namespace luhsoccer::python {

class StaticObserver {};
class StaticObserverPosition {};

using namespace game_data_provider;
using namespace observer;

template <>
void bindModule(nb::module_& baguette_module, nb::class_<GameDataProvider>& instance) {
    loadEnumBindings<StrategyType>(baguette_module, "StrategyType");
    loadClassBindings<AllyRobot::BestPassReceiver>(baguette_module, "BestPassReceiver");
    loadClassBindings<BallHolder>(baguette_module, "BallHolder");
    loadClassBindings<Observer>(baguette_module, "Observer");

    loadClassBindings<StaticObserver>(baguette_module, "StaticObserver");
    loadClassBindings<StaticObserverPosition>(baguette_module, "StaticObserverPosition");

    loadClassBindings<TeamInfo>(baguette_module, "TeamInfo");

    loadDerivedClassBindings<GameStateChangedEvent, event_system::Event>(baguette_module, "GameStateChangedEvent");
    loadDerivedClassBindings<GameStageChangedEvent, event_system::Event>(baguette_module, "GameStageChangedEvent");
    loadDerivedClassBindings<RealWorldModelUpdatedEvent, event_system::Event>(baguette_module,
                                                                              "RealWorldModelUpdatedEvent");
    loadDerivedClassBindings<FieldDataUpdatedEvent, event_system::Event>(baguette_module, "FieldDataUpdatedEvent");

    loadDerivedClassBindings<ThreatLevelChangedEvent, event_system::Event>(baguette_module, "ThreatLevelChangedEvent");
    loadDerivedClassBindings<BestInterceptorChangedEvent, event_system::Event>(baguette_module,
                                                                               "BestInterceptorChangedEvent");
    loadDerivedClassBindings<DominantTeamChangeEvent, event_system::Event>(baguette_module, "DominantTeamChangeEvent");
    loadDerivedClassBindings<BallCarrierChangedEvent, event_system::Event>(baguette_module, "BallCarrierChangedEvent");
    loadDerivedClassBindings<RobotMovedEvent, event_system::Event>(baguette_module, "RobotMovedEvent");
    loadDerivedClassBindings<RobotRemovedFromFieldEvent, event_system::Event>(baguette_module,
                                                                              "RobotRemovedFromFieldEvent");

    loadDerivedClassBindings<BallShotEvent, event_system::Event>(baguette_module, "BallShotEvent");

    instance.def("getWorldModel", &GameDataProvider::getWorldModel, nb::rv_policy::copy);
    instance.def("getObserver", &GameDataProvider::getObserver, nb::rv_policy::reference);
    instance.def("getSpecialKickPosition", &GameDataProvider::getSpecialKickPosition);
    instance.def("getSpecialKickTime", &GameDataProvider::getSpecialKickTime);
    instance.def("getGoalie", [](const GameDataProvider& gdp) {
        return transform::RobotHandle(gdp.getGoalie(), gdp.getWorldModel());
    });

    instance.def("getGameStage", &GameDataProvider::getGameStage);

    instance.def("getEnemyGoalie", [](const GameDataProvider& gdp) {
        return transform::RobotHandle(gdp.getEnemyGoalie(), gdp.getWorldModel());
    });

    instance.def("getAllyTeamInfo", &GameDataProvider::getAllyTeamInfo);
    instance.def("getEnemyTeamInfo", &GameDataProvider::getEnemyTeamInfo);

    instance.def_ro_static("ball_frame", &GameDataProvider::BALL_FRAME);
    instance.def_ro_static("ball_placement_frame", &GameDataProvider::BALL_PLACEMENT_FRAME);
    instance.def_ro_static("ball_position", &GameDataProvider::BALL_POSITION);
    instance.def_ro_static("ball_placement_position", &GameDataProvider::BALL_PLACEMENT_POSITION);
}

template <>
void bindClass(nb::class_<AllyRobot::BestPassReceiver>& instance) {
    instance.def_ro("handle", &AllyRobot::BestPassReceiver::handle);
    instance.def_ro("score", &AllyRobot::BestPassReceiver::score);
}

template <>
void bindClass(nb::class_<BallHolder>& instance) {
    instance.def_ro("handle", &BallHolder::handle);
    instance.def_ro("ball_obtained_pos", &BallHolder::ball_obtained_pos);
    instance.def_ro("movement_allowed", &BallHolder::movement_allowed);
}

template <>
void bindClass(nb::class_<Observer>& instance) {
    instance.def("getBallCarrier", &Observer::getBallCarrier);
    instance.def("getBallControllingTeam", &Observer::getBallControllingTeam);
    instance.def("getBallGoalProbability", &Observer::getBallGoalProbability);
    instance.def("getBallObtainedPos", &Observer::getBallObtainedPos);
    instance.def("getGoalProbability", &Observer::getGoalProbability);
    instance.def("getThreatLevel", &Observer::getThreatLevel);
    instance.def("getBestPassReceiver", &Observer::getBestPassReceiver);
    instance.def("getStrategyType", &Observer::getStrategyType);
    instance.def("getLastBallTouchingRobot", &Observer::getLastBallTouchingRobot);
    instance.def("getPotentialDoubleToucher", &Observer::getPotentialDoubleToucher);
}

template <>
void bindDerivedClass(nb::class_<ThreatLevelChangedEvent, event_system::Event>& instance) {
    instance.def_ro("robot", &ThreatLevelChangedEvent::robot);
    instance.def_ro("old_threat_level", &ThreatLevelChangedEvent::old_threat_level);
    instance.def_ro("new_threat_level", &ThreatLevelChangedEvent::new_threat_level);
}

template <>
void bindDerivedClass(nb::class_<DominantTeamChangeEvent, event_system::Event>& instance) {
    instance.def_ro("dominant_team", &DominantTeamChangeEvent::dominant_team);
}

template <>
void bindDerivedClass(nb::class_<RobotMovedEvent, event_system::Event>& instance) {
    instance.def_ro("handle", &RobotMovedEvent::handle);
    instance.def_ro("distance", &RobotMovedEvent::distance);
    instance.def_ro("old_position", &RobotMovedEvent::old_position);
    instance.def_ro("new_position", &RobotMovedEvent::new_position);
}

template <>
void bindDerivedClass(nb::class_<BallCarrierChangedEvent, event_system::Event>& instance) {
    instance.def_ro("old_carrier", &BallCarrierChangedEvent::old_carrier);
    instance.def_ro("new_carrier", &BallCarrierChangedEvent::new_carrier);
}

/**
 * @brief All Static Observer Methods which take a RobotHandle or a WorldModel
 *
 */
template <>
void bindClass(nb::class_<StaticObserver>& instance) {
    namespace calc = observer::calculation;

    instance.def_static("calculateBallPosession", &calc::calculateBallPosession, nb::arg("gdp"));

    instance.def_static(
        "calculateGoalProbability",
        nb::overload_cast<const transform::RobotHandle&, const time::TimePoint>(&calc::calculateGoalProbability),
        nb::arg("handle"), nb::arg("time") = time::TimePoint(0));

    instance.def_static(
        "calculateShootPoint",
        nb::overload_cast<const transform::RobotHandle&, const time::TimePoint>(&calc::calculateShootPoint),
        nb::arg("handle"), nb::arg("time") = time::TimePoint(0));

    instance.def_static("calculateThreatLevel",
                        nb::overload_cast<const transform::RobotHandle&>(&calc::calculateThreatLevel),
                        nb::arg("handle"));

    instance.def_static(
        "calculateBestPassReceiver",
        nb::overload_cast<const transform::RobotHandle&, std::vector<RobotIdentifier>, const time::TimePoint>(
            &calc::calculateBestPassReceiver),
        nb::arg("handle"), nb::arg("allies_to_evaluate") = std::vector<RobotIdentifier>{},
        nb::arg("time") = time::TimePoint(0));

    instance.def_static("calculatePassProbability",
                        nb::overload_cast<const transform::RobotHandle&, const RobotIdentifier&,
                                          const std::shared_ptr<const transform::WorldModel>&, const time::TimePoint>(
                            &calc::calculatePassProbability),
                        nb::arg("passer"), nb::arg("receiver"), nb::arg("world_model"),
                        nb::arg("time") = time::TimePoint(0));

    instance.def_static(
        "isPassLineDefended",
        nb::overload_cast<const transform::RobotHandle&, const transform::RobotHandle&, const time::TimePoint>(
            &calc::isPassLineDefended),
        nb::arg("handle"), nb::arg("other_handle"), nb::arg("time") = time::TimePoint(0));

    instance.def_static(
        "isOutgoingPassDefended",
        nb::overload_cast<const transform::RobotHandle&, const time::TimePoint>(&calc::isOutgoingPassDefended),
        nb::arg("handle"), nb::arg("time") = time::TimePoint(0));

    instance.def_static(
        "calculateBestGoalPoint",
        nb::overload_cast<const transform::RobotHandle&, std::optional<RobotIdentifier>, const time::TimePoint>(
            &calc::calculateBestGoalPoint),
        nb::arg("handle"), nb::arg("id_to_ignorer") = std::nullopt, nb::arg("time") = time::TimePoint(0));

    instance.def_static("calculateMostLikelyShootPoint",
                        nb::overload_cast<const transform::RobotHandle&, const game_data_provider::GameDataProvider&,
                                          const time::TimePoint>(&calculation::calculateMostLikelyShootPoint),
                        nb::arg("enemy"), nb::arg("gdp"), nb::arg("time") = time::TimePoint(0));

    instance.def_static("calculateFutureBallPos", &calc::calculateFutureBallPos, nb::arg("world_model"),
                        nb::arg("time") = time::TimePoint(0));

    instance.def_static("calculateInterceptionRobotIsViable",
                        nb::overload_cast<const transform::RobotHandle&, const time::TimePoint>(
                            &calculation::calculateInterceptionRobotIsViable),
                        nb::arg("handle"), nb::arg("time") = time::TimePoint(0));

    instance.def_static(
        "calculateBestInterceptor",
        nb::overload_cast<const std::shared_ptr<const transform::WorldModel>&, std::vector<RobotIdentifier>,
                          const time::TimePoint>(&calculation::calculateBestInterceptor),
        nb::arg("world_model"), nb::arg("viable_interceptors") = std::vector<RobotIdentifier>{},
        nb::arg("time") = time::TimePoint(0));

    instance.def_static("calculateInterceptionScore", &calculation::calculateInterceptionScore, nb::arg("handle"),
                        nb::arg("time") = time::TimePoint(0));
}

/**
 * @brief All Static Observer Methods which take a transform::Position
 *
 */
template <>
void bindClass(nb::class_<StaticObserverPosition>& instance) {
    namespace calc = observer::calculation;

    instance.def_static(
        "calculateGoalProbability",
        nb::overload_cast<const transform::Position&, Team, std::shared_ptr<const transform::WorldModel>,
                          const time::TimePoint>(&calc::calculateGoalProbability),
        nb::arg("pos"), nb::arg("team"), nb::arg("world_model"), nb::arg("time") = time::TimePoint(0));

    instance.def_static(
        "calculateBestGoalPoint",
        nb::overload_cast<const transform::Position&, Team, std::shared_ptr<const transform::WorldModel>,
                          std::optional<RobotIdentifier>, const time::TimePoint>(&calc::calculateBestGoalPoint),
        nb::arg("pos"), nb::arg("team"), nb::arg("world_model"), nb::arg("id_to_ignore") = std::nullopt,
        nb::arg("time") = time::TimePoint(0));

    instance.def_static("calculateThreatLevel",
                        nb::overload_cast<const transform::Position&, std::shared_ptr<const transform::WorldModel>,
                                          const time::TimePoint>(&calc::calculateThreatLevel),
                        nb::arg("pos"), nb::arg("world_model"), nb::arg("time") = time::TimePoint(0));

    instance.def_static("isPassLineDefended",
                        nb::overload_cast<const transform::Position&, const transform::Position&, const Team,
                                          std::shared_ptr<const transform::WorldModel>, const time::TimePoint>(
                            &calc::isPassLineDefended),
                        nb::arg("pos"), nb::arg("other_pos"), nb::arg("team"), nb::arg("world_model"),
                        nb::arg("time") = time::TimePoint(0));

    instance.def_static(
        "isOutgoingPassDefended",
        nb::overload_cast<const transform::Position&, Team, std::shared_ptr<const transform::WorldModel>,
                          std::optional<RobotIdentifier>, const time::TimePoint>(&calc::isOutgoingPassDefended),
        nb::arg("robot_pos"), nb::arg("team"), nb::arg("world_model"), nb::arg("robot_id") = std::nullopt,
        nb::arg("time") = time::TimePoint(0));

    instance.def_static(
        "calculateBestPassReceiver",
        nb::overload_cast<const transform::Position&, Team, std::shared_ptr<const transform::WorldModel>,
                          std::vector<RobotIdentifier>, std::optional<RobotIdentifier>, const time::TimePoint>(
            &calc::calculateBestPassReceiver),
        nb::arg("pos"), nb::arg("team"), nb::arg("world_model"),
        nb::arg("allies_to_evaluate") = std::vector<RobotIdentifier>{}, nb::arg("passing_robot") = std::nullopt,
        nb::arg("time") = time::TimePoint(0));

    instance.def_static("calculatePassProbability",
                        nb::overload_cast<const transform::Position&, const transform::Position&, const Team,
                                          const std::shared_ptr<const transform::WorldModel>&, const time::TimePoint>(
                            &calc::calculatePassProbability),
                        nb::arg("passer_pos"), nb::arg("receiver_pos"), nb::arg("team"), nb::arg("world_model"),
                        nb::arg("time") = time::TimePoint(0));

    instance.def_static(
        "calculateShootPoint",
        nb::overload_cast<const transform::Position&, Team, std::shared_ptr<const transform::WorldModel>,
                          const time::TimePoint>(&calc::calculateShootPoint),
        nb::arg("pos"), nb::arg("team"), nb::arg("world_model"), nb::arg("time") = time::TimePoint(0));

    instance.def_static("calculatePassSuccessChance", &calculation::calcPassSuccessChance, nb::arg("start_pos"),
                        nb::arg("end_pos"), nb::arg("kicker_pos"), nb::arg("enemies"), nb::arg("time"),
                        nb::arg("pass_power"), nb::arg("enemy_max_speed"), nb::arg("radius_scaling"),
                        nb::arg("pass_power_lut"), nb::arg("turn_time_lut"), nb::arg("pass_time_lut"));
}

template <>
void bindClass(nb::class_<TeamInfo>& instance) {
    instance.def_ro("ball_placement_failure", &TeamInfo::ball_placement_failures);
    instance.def_ro("ball_placement_failures_reached", &TeamInfo::ball_placement_failures_reached);
    instance.def_ro("can_place_ball", &TeamInfo::can_place_ball);
    instance.def_ro("bot_substitution_intent", &TeamInfo::bot_substitution_intent);
    instance.def_ro("foul_counter", &TeamInfo::foul_counter);
    instance.def_ro("goalkeeper", &TeamInfo::goalkeeper);
    instance.def_ro("max_allowed_bots", &TeamInfo::max_allowed_bots);
    instance.def_ro("name", &TeamInfo::name);
    instance.def_ro("red_cards", &TeamInfo::red_cards);
    instance.def_ro("yellow_cards", &TeamInfo::yellow_cards);
    instance.def_ro("score", &TeamInfo::score);
    instance.def_ro("yellow_card_times", &TeamInfo::yellow_card_times);
    instance.def_ro("timeout_time", &TeamInfo::timeout_time);
    instance.def_ro("timeouts", &TeamInfo::timeouts);
}

template <>
void bindDerivedClass(nb::class_<GameStateChangedEvent, event_system::Event>& instance) {
    instance.def_ro("old_state", &GameStateChangedEvent::old_state);
    instance.def_ro("new_state", &GameStateChangedEvent::new_state);
}

template <>
void bindDerivedClass(nb::class_<GameStageChangedEvent, event_system::Event>& instance) {
    instance.def_ro("state", &GameStageChangedEvent::stage);
}

template <>
void bindDerivedClass(nb::class_<RealWorldModelUpdatedEvent, event_system::Event>& instance) {
    instance.def_ro("world_model", &RealWorldModelUpdatedEvent::world_model);
    instance.def_ro("timestamp_capture", &RealWorldModelUpdatedEvent::timestamp_capture);
    instance.def_ro("timestamp_sent", &RealWorldModelUpdatedEvent::timestamp_sent);
}

template <>
void bindDerivedClass(nb::class_<FieldDataUpdatedEvent, event_system::Event>& instance) {
    instance.def_ro("field_data", &FieldDataUpdatedEvent::field_data);
}

template <>
void bindDerivedClass(nb::class_<RobotRemovedFromFieldEvent, event_system::Event>& instance) {
    instance.def_ro("robot", &RobotRemovedFromFieldEvent::robot);
}

template <>
void bindDerivedClass(nb::class_<BestInterceptorChangedEvent, event_system::Event>& instance) {
    instance.def_ro("old_interceptor", &BestInterceptorChangedEvent::old_interceptor);
    instance.def_ro("new_interceptor", &BestInterceptorChangedEvent::new_interceptor);
}

template <>
void bindDerivedClass(nb::class_<BallShotEvent, event_system::Event>& instance) {
    instance.def_ro("old_velocity", &BallShotEvent::old_velocity);
    instance.def_ro("new_velocity", &BallShotEvent::new_velocity);
}

}  // namespace luhsoccer::python