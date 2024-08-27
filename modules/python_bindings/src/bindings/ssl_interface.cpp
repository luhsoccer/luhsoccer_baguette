#include "bindings.hpp"

namespace luhsoccer::python {

using namespace ssl_interface;

template <>
void bindModule(nb::module_& baguette_module, nb::class_<SSLInterface>& instance) {
    loadEnumBindings<VisionDataSource>(baguette_module, "VisionDataSource");
    loadEnumBindings<GameControllerDataSource>(baguette_module, "GamecontrollerDataSource");
    loadEnumBindings<VisionPublishMode>(baguette_module, "VisionPublishMode");

    loadEnumBindings<SSLLineType>(baguette_module, "SSLLineType");
    loadEnumBindings<SSLArcType>(baguette_module, "SSLArcType");

    loadClassBindings<SSLRobotInfo>(baguette_module, "SSLRobotInfo");
    loadClassBindings<SSLBallInfo>(baguette_module, "SSLBallInfo");
    loadClassBindings<SSLVisionData>(baguette_module, "SSLVisionData");

    loadClassBindings<SSLFieldLine>(baguette_module, "SSLFieldLine");
    loadClassBindings<SSLFieldArc>(baguette_module, "SSLFieldArc");
    loadClassBindings<SSLFieldData>(baguette_module, "SSLFieldData");

    loadDerivedClassBindings<NewVisionDataEvent, event_system::Event>(baguette_module, "NewVisionDataEvent");
    loadDerivedClassBindings<NewFieldDataEvent, event_system::Event>(baguette_module, "NewFieldDataEvent");
    loadDerivedClassBindings<NewGameControllerDataEvent, event_system::Event>(baguette_module,
                                                                              "NewGameControllerDataEvent");

    instance.def("setVisionDataSource", &SSLInterface::setVisionDataSource);
    instance.def("getVisionDataSource", &SSLInterface::getVisionDataSource);
    instance.def("setGameControllerDataSource", &SSLInterface::setGameControllerDataSource);
    instance.def("getGameControllerDataSource", &SSLInterface::getGameControllerDataSource);
    instance.def("setVisionPublishMode", &SSLInterface::setVisionPublishMode);
    instance.def("getVisionPublishMode", &SSLInterface::getVisionPublishMode);
}

template <>
void bindClass(nb::class_<SSLFieldLine>& instance) {
    instance.def_ro("type", &SSLFieldLine::type);
    instance.def_ro("name", &SSLFieldLine::name);
    instance.def_ro("start_point", &SSLFieldLine::start_point);
    instance.def_ro("end_point", &SSLFieldLine::end_point);
    instance.def_ro("thickness", &SSLFieldLine::thickness);
}

template <>
void bindClass(nb::class_<SSLFieldArc>& instance) {
    instance.def_ro("type", &SSLFieldArc::type);
    instance.def_ro("name", &SSLFieldArc::name);
    instance.def_ro("center", &SSLFieldArc::center);
    instance.def_ro("radius", &SSLFieldArc::radius);
    instance.def_ro("start_angle", &SSLFieldArc::start_angle);
    instance.def_ro("end_angle", &SSLFieldArc::end_angle);
    instance.def_ro("thickness", &SSLFieldArc::thickness);
}

template <>
void bindClass(nb::class_<SSLFieldData>& instance) {
    instance.def_ro("size", &SSLFieldData::size);
    instance.def_ro("goal_width", &SSLFieldData::goal_width);
    instance.def_ro("goal_depth", &SSLFieldData::goal_depth);
    instance.def_ro("boundary_width", &SSLFieldData::boundary_width);
    instance.def_ro("penalty_area_depth", &SSLFieldData::penalty_area_depth);
    instance.def_ro("penalty_area_width", &SSLFieldData::penalty_area_width);
    instance.def_ro("center_circle_radius", &SSLFieldData::center_circle_radius);
    instance.def_ro("line_thickness", &SSLFieldData::line_thickness);
    instance.def_ro("goal_center_to_penalty_mark", &SSLFieldData::goal_center_to_penalty_mark);
    instance.def_ro("goal_height", &SSLFieldData::goal_height);
    instance.def_ro("ball_radius", &SSLFieldData::ball_radius);
    instance.def_ro("max_robot_radius", &SSLFieldData::max_robot_radius);
    instance.def_ro("lines", &SSLFieldData::lines);
    instance.def_ro("arcs", &SSLFieldData::arcs);
    instance.def_ro("field_left_top", &SSLFieldData::field_left_top);
    instance.def_ro("field_left_bottom", &SSLFieldData::field_left_bottom);
    instance.def_ro("field_right_top", &SSLFieldData::field_right_top);
    instance.def_ro("field_right_bottom", &SSLFieldData::field_right_bottom);
    instance.def_ro("field_left_center", &SSLFieldData::field_left_center);
    instance.def_ro("field_right_center", &SSLFieldData::field_right_center);
    instance.def_ro("field_center", &SSLFieldData::field_center);
    instance.def_ro("field_top_center", &SSLFieldData::field_top_center);
    instance.def_ro("field_bottom_center", &SSLFieldData::field_bottom_center);
    instance.def_ro("penalty_area_left_baseline_top", &SSLFieldData::penalty_area_left_baseline_top);
    instance.def_ro("penalty_area_right_baseline_top", &SSLFieldData::penalty_area_right_baseline_top);
    instance.def_ro("penalty_area_left_baseline_bottom", &SSLFieldData::penalty_area_left_baseline_bottom);
    instance.def_ro("penalty_area_right_baseline_bottom", &SSLFieldData::penalty_area_right_baseline_bottom);
    instance.def_ro("penalty_area_left_field_top", &SSLFieldData::penalty_area_left_field_top);
    instance.def_ro("penalty_area_right_field_top", &SSLFieldData::penalty_area_right_field_top);
    instance.def_ro("penalty_area_left_field_bottom", &SSLFieldData::penalty_area_left_field_bottom);
    instance.def_ro("penalty_area_right_field_bottom", &SSLFieldData::penalty_area_right_field_bottom);
    instance.def_ro("penalty_area_left_field_center", &SSLFieldData::penalty_area_left_field_center);
    instance.def_ro("penalty_area_right_field_center", &SSLFieldData::penalty_area_right_field_center);
    instance.def_ro("goal_left_top", &SSLFieldData::goal_left_top);
    instance.def_ro("goal_left_bottom", &SSLFieldData::goal_left_bottom);
    instance.def_ro("goal_right_top", &SSLFieldData::goal_right_top);
    instance.def_ro("goal_right_bottom", &SSLFieldData::goal_right_bottom);
}

template <>
void bindClass(nb::class_<SSLRobotInfo>& instance) {
    instance.def_ro("id", &SSLRobotInfo::id);
    instance.def_ro("transform", &SSLRobotInfo::transform);
    instance.def_ro("confidence", &SSLRobotInfo::confidence);
    instance.def_ro("pixel_position", &SSLRobotInfo::pixel_position);
}

template <>
void bindClass(nb::class_<SSLBallInfo>& instance) {
    instance.def_ro("position", &SSLBallInfo::position);
    instance.def_ro("confidence", &SSLBallInfo::confidence);
    instance.def_ro("pixel_position", &SSLBallInfo::pixel_position);
}

template <>
void bindClass(nb::class_<SSLVisionData>& instance) {
    instance.def_ro("blue_robots", &SSLVisionData::blue_robots);
    instance.def_ro("yellow_robots", &SSLVisionData::yellow_robots);
    instance.def_ro("balls", &SSLVisionData::balls);
    instance.def_ro("timestamp_capture", &SSLVisionData::timestamp_capture);
    instance.def_ro("timestamp_sent", &SSLVisionData::timestamp_sent);
    instance.def_ro("frame_number", &SSLVisionData::frame_number);
    instance.def_ro("camera_id", &SSLVisionData::camera_id);
}

template <>
void bindDerivedClass(nb::class_<NewVisionDataEvent, event_system::Event>& instance) {
    instance.def_ro("data", &NewVisionDataEvent::data);
}

template <>
void bindDerivedClass(nb::class_<NewFieldDataEvent, event_system::Event>& instance) {
    instance.def_ro("data", &NewFieldDataEvent::data);
}

template <>
void bindDerivedClass(nb::class_<NewGameControllerDataEvent, event_system::Event>& instance) {
    instance.def_ro("data", &NewGameControllerDataEvent::data);
}

}  // namespace luhsoccer::python