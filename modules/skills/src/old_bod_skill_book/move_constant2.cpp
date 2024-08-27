#include "skill_books/bod_skill_book/move_constant2.hpp"
// include components here
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/shapes/point_shape.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"

namespace luhsoccer::skills {

MoveConstant2Build::MoveConstant2Build()
    : SkillBuilder("MoveConstant2",                         //
                   {},                                      //
                   {"TargetPosition1", "TargetPosition2"},  //
                   {"TargetDuration1", "TargetDuration2"},  //
                   {},                                      //
                   {},                                      //
                   {}){};

void MoveConstant2Build::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    DriveStep d;
    d.addFeature(TargetFeature(PointShape({TD_Pos::POINT, 0})));
    d.setRotationControl(HeadingRotationControl(0.0, {TD_Pos::POINT, 0}));
    d.setAvoidOtherRobots(false);
    d.setAvoidDefenseArea(false);
    DoubleComponentParam max_speed(CALLBACK, [](const CallbackData& data) -> double {
        auto robot_to_goal = data.td.required_positions[0].getCurrentPosition(data.wm, data.td.robot.getFrame());
        if (!robot_to_goal.has_value()) return 0.0;
        double distance_to_target = robot_to_goal->translation().norm();
        auto start_time = data.td.getCookie<time::TimePoint>(data.component_uid, "start_time");
        if (!start_time.has_value()) {
            start_time = time::now();
            data.td.setCookie(data.component_uid, "start_time", start_time.value());
        }
        double time_to_target = data.td.required_doubles[0] + start_time.value().asSec() - time::now().asSec();
        return std::max(distance_to_target / time_to_target, 0.0);
    });
    d.setMaxVelX(max_speed);
    d.setMaxVelY(max_speed);
    d.setCancelCondition({CALLBACK, [](const CallbackData& data) {
                              auto start_time = data.td.getCookie<time::TimePoint>(data.component_uid, "start_time");
                              if (!start_time.has_value()) {
                                  start_time = time::now();
                                  data.td.setCookie(data.component_uid, "start_time", start_time.value());
                              }
                              return data.td.required_doubles[0] + start_time.value().asSec() < time::now().asSec();
                          }});
    addStep(std::move(d));

    DriveStep d2;
    d2.addFeature(TargetFeature(PointShape({TD_Pos::POINT, 1})));
    d2.setRotationControl(HeadingRotationControl(0.0, {TD_Pos::POINT, 1}));
    d2.setAvoidOtherRobots(false);
    d2.setAvoidDefenseArea(false);
    DoubleComponentParam max_speed2(CALLBACK, [](const CallbackData& data) -> double {
        auto robot_to_goal = data.td.required_positions[1].getCurrentPosition(data.wm, data.td.robot.getFrame());
        if (!robot_to_goal.has_value()) return 0.0;
        double distance_to_target = robot_to_goal->translation().norm();
        auto start_time = data.td.getCookie<time::TimePoint>(data.component_uid, "start_time");
        if (!start_time.has_value()) {
            start_time = time::now();
            data.td.setCookie(data.component_uid, "start_time", start_time.value());
        }
        double time_to_target = data.td.required_doubles[1] + start_time.value().asSec() - time::now().asSec();
        return std::max(distance_to_target / time_to_target, 0.0);
    });
    d2.setMaxVelX(max_speed2);
    d2.setMaxVelY(max_speed2);
    d2.setCancelCondition({CALLBACK, [](const CallbackData& data) {
                               auto start_time = data.td.getCookie<time::TimePoint>(data.component_uid, "start_time");
                               if (!start_time.has_value()) {
                                   start_time = time::now();
                                   data.td.setCookie(data.component_uid, "start_time", start_time.value());
                               }
                               return data.td.required_doubles[1] + start_time.value().asSec() < time::now().asSec();
                           }});
    addStep(std::move(d2));
    // end of skill
}
}  // namespace luhsoccer::skills