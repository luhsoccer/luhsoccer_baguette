#include "bindings.hpp"

namespace luhsoccer::python {

using namespace marker;

template <>
void bindModule(py::module_& baguette_module, py::class_<MarkerService>& instance) {
    loadEnumBindings<MType>(baguette_module, "MarkerType");
    loadClassBindings<Color>(baguette_module, "Color");
    loadClassBindings<ScaleVec3>(baguette_module, "ScaleVec3");
    loadClassBindings<SizeVec2>(baguette_module, "SizeVec2");
    loadClassBindings<Point>(baguette_module, "Point");
    loadClassBindings<Marker>(baguette_module, "Marker");
    loadDerivedClassBindings<Marker2D, Marker>(baguette_module, "Marker2D");
    loadDerivedClassBindings<GoalBorder, Marker>(baguette_module, "GoalBorderMarker");
    loadDerivedClassBindings<Robot, Marker>(baguette_module, "RobotMarker");
    loadDerivedClassBindings<Ball, Marker>(baguette_module, "BallMarker");
    loadDerivedClassBindings<Cone, Marker>(baguette_module, "ConeMarker");
    loadDerivedClassBindings<Cube, Marker>(baguette_module, "CubeMarker");
    loadDerivedClassBindings<Sphere, Marker>(baguette_module, "SphereMarker");
    loadDerivedClassBindings<Torus, Marker>(baguette_module, "TorusMarker");
    loadDerivedClassBindings<Arrow, Marker>(baguette_module, "ArrowMarker");
    loadDerivedClassBindings<Suzanne, Marker>(baguette_module, "SuzanneMarker");
    loadDerivedClassBindings<Text, Marker>(baguette_module, "TextMarker");
    loadDerivedClassBindings<Circle, Marker2D>(baguette_module, "CircleMarker");
    loadDerivedClassBindings<Rect, Marker2D>(baguette_module, "RectMarker");
    loadDerivedClassBindings<Line, Marker2D>(baguette_module, "LineMarker");
    loadDerivedClassBindings<Arrow2d, Marker2D>(baguette_module, "Arrow2dMarker");
    loadDerivedClassBindings<LineStrip, Marker2D>(baguette_module, "LineStripMarker");
    loadDerivedClassBindings<Heatmap, Marker2D>(baguette_module, "HeatmapMarker");
    loadDerivedClassBindings<CircularHeatmap, Marker2D>(baguette_module, "CircularHeatmapMarker");
    loadDerivedClassBindings<CustomStrip, Marker>(baguette_module, "CustomStripMarker");
    loadDerivedClassBindings<Info, Marker>(baguette_module, "InfoMarker");
    loadDerivedClassBindings<RobotInfo, Marker>(baguette_module, "RobotInfoMarker");

    instance.def("displayMarker", py::overload_cast<Marker>(&MarkerService::displayMarker));
    instance.def("displayRobotInfoMarker", py::overload_cast<RobotInfo>(&MarkerService::displayMarker));
    instance.def("displayInfoMarker", py::overload_cast<Info>(&MarkerService::displayMarker));
    instance.def("deleteMarker", &MarkerService::deleteMarker);
}

template <>
void bindEnum(py::enum_<MType>& instance) {
    instance.value("GoalBorder", MType::GOAL_BORDERS);
    instance.value("Robot", MType::ROBOT);
    instance.value("Ball", MType::BALL);
    instance.value("Frame", MType::FRAME);
    instance.value("Cone", MType::CONE);
    instance.value("Cube", MType::CUBE);
    instance.value("Cylinder", MType::CYLINDER);
    instance.value("Sphere", MType::SPHERE);
    instance.value("Torus", MType::TORUS);
    instance.value("Arrow", MType::ARROW);
    instance.value("Suzanne", MType::SUZANNE);
    instance.value("Text", MType::TEXT);
    instance.value("Line", MType::LINE);
    instance.value("Rect", MType::RECT);
    instance.value("Arrow2d", MType::ARROW_2D);
    instance.value("Circle", MType::CIRCLE);
    instance.value("LineStrip", MType::LINE_STRIP);
    instance.value("CustomStrip", MType::CUSTOM_STRIP);
    instance.value("HeatMap", MType::HEATMAP);
    instance.value("CircleHeatmap", MType::CIRCULAR_HEATMAP);
}

template <>
void bindClass(py::class_<Color>& instance) {
    instance.def(py::init<double, double, double>());
    instance.def_static("red", &Color::RED);
    instance.def_static("green", &Color::GREEN);
    instance.def_static("blue", &Color::BLUE);
    instance.def_static("yellow", &Color::YELLOW);
    instance.def_static("orange", &Color::ORANGE);
    instance.def_static("lightGreen", &Color::LIGHT_GREEN);
    instance.def_static("lightBlue", &Color::LIGHT_BLUE);
    instance.def_static("purple", &Color::PURPLE);
    instance.def_static("pink", &Color::PINK);
    instance.def_static("grey", &Color::GREY);
    instance.def_static("lightGrey", &Color::LIGHT_GREY);
    instance.def_static("white", &Color::WHITE);
    instance.def_static("black", &Color::BLACK);

    instance.def_static("random", py::overload_cast<unsigned int>(&Color::random));
    instance.def_static("random", py::overload_cast<>(&Color::random));
    instance.def_static("interpolate", &Color::interpolate);
    instance.def_static("interpolateGradient", &Color::interpolateGradient);
    instance.def_static("hsv2rgb", &Color::hsv2Rgb);
}

template <>
void bindClass(py::class_<ScaleVec3>& instance) {
    instance.def(py::init<double>());
    instance.def(py::init<double, double, double>());
}

template <>
void bindClass(py::class_<SizeVec2>& instance) {
    instance.def(py::init<double>());
    instance.def(py::init<double, double>());
}

template <>
void bindClass(py::class_<Point>& instance) {
    instance.def(py::init<>());
    instance.def(py::init<double, double, double>());
}

template <>
void bindClass(py::class_<Marker>& instance) {
    instance.def("setNs", &Marker::setNs);
    instance.def("setId", &Marker::setId);
    instance.def("setColor", &Marker::setColor);
    instance.def("setScale", &Marker::setScale);
    instance.def("setLifetime", &Marker::setLifetime);
    instance.def("setFrameLocked", &Marker::setFrameLocked);
    instance.def("setHeight", &Marker::setHeight);
    instance.def("getNs", &Marker::getNs);
    instance.def("getId", &Marker::getId);
}

template <>
void bindDerivedClass(py::class_<Marker2D, Marker>& instance) {
    instance.def("setThickness", &Marker2D::setThickness);
    instance.def("setColors", &Marker2D::setColors);
}

template <>
void bindDerivedClass(py::class_<GoalBorder, Marker>& instance) {
    instance.def(py::init<transform::Position, std::string, size_t>());
}

template <>
void bindDerivedClass(py::class_<Robot, Marker>& instance) {
    instance.def(py::init<transform::Position, const RobotIdentifier&, std::string, size_t>());
}

template <>
void bindDerivedClass(py::class_<Ball, Marker>& instance) {
    instance.def(py::init<transform::Position, std::string, size_t>());
}

template <>
void bindDerivedClass(py::class_<Cone, Marker>& instance) {
    instance.def(py::init<transform::Position, std::string, size_t>());
}

template <>
void bindDerivedClass(py::class_<Cube, Marker>& instance) {
    instance.def(py::init<transform::Position, std::string, size_t>());
}

template <>
void bindDerivedClass(py::class_<Cylinder, Marker>& instance) {
    instance.def(py::init<transform::Position, std::string, size_t>());
}

template <>
void bindDerivedClass(py::class_<Sphere, Marker>& instance) {
    instance.def(py::init<transform::Position, std::string, size_t>());
}

template <>
void bindDerivedClass(py::class_<Torus, Marker>& instance) {
    instance.def(py::init<transform::Position, std::string, size_t>());
}

template <>
void bindDerivedClass(py::class_<Arrow, Marker>& instance) {
    instance.def(py::init<transform::Position, std::string, size_t>());
}

template <>
void bindDerivedClass(py::class_<Suzanne, Marker>& instance) {
    instance.def(py::init<transform::Position, std::string, size_t>());
}

template <>
void bindDerivedClass(py::class_<Text, Marker>& instance) {
    instance.def(py::init<transform::Position, std::string, size_t, std::string>());
    instance.def("setText", &Text::setText);
}

template <>
void bindDerivedClass(py::class_<Circle, Marker2D>& instance) {
    instance.def(py::init<transform::Position, std::string, size_t>());
    instance.def("setRadius", &Circle::setRadius);
    instance.def("setFilled", &Circle::setFilled);
}

template <>
void bindDerivedClass(py::class_<Rect, Marker2D>& instance) {
    instance.def(py::init<transform::Position, std::string, size_t>());
    instance.def("setSize", &Rect::setSize);
    instance.def("setFilled", &Rect::setFilled);
}

template <>
void bindDerivedClass(py::class_<Line, Marker2D>& instance) {
    instance.def(py::init<transform::Position, std::string, size_t>());
    instance.def("setLinePoints", &Line::setLinePoints);
}

template <>
void bindDerivedClass(py::class_<Arrow2d, Marker2D>& instance) {
    instance.def(py::init<transform::Position, std::string, size_t>());
    instance.def("setSize", &Arrow2d::setSize);
}

template <>
void bindDerivedClass(py::class_<LineStrip, Marker2D>& instance) {
    instance.def(py::init<transform::Position, std::string, size_t>());
    instance.def("setPathClosed", &LineStrip::setPathClosed);
    instance.def("setPoints", &LineStrip::setPoints);
}

template <>
void bindDerivedClass(py::class_<Heatmap, Marker2D>& instance) {
    instance.def(py::init<transform::Position, std::string, size_t>());
    instance.def("setHeatmapColors", &Heatmap::setHeatmapColors);
    instance.def("setHeatmapSize", &Heatmap::setHeatmapSize);
    instance.def("setHeatmapPoints", &Heatmap::setHeatmapPoints);
    instance.def("setHeatmapMinMax", &Heatmap::setHeatmapMinMax);
    instance.def("setHeatmapGradient", &Heatmap::setHeatmapGradient);
}

template <>
void bindDerivedClass(py::class_<CircularHeatmap, Marker2D>& instance) {
    instance.def(py::init<transform::Position, std::string, size_t>());
    instance.def("setCircularHeatmapColors", &CircularHeatmap::setCircularHeatmapColors);
    instance.def("setRadius", &CircularHeatmap::setRadius);
    instance.def("setCircularHeatmapData", &CircularHeatmap::setCircularHeatmapData);
    instance.def("setHeatmapMinMax", &CircularHeatmap::setHeatmapMinMax);
    instance.def("setHeatmapGradient", &CircularHeatmap::setHeatmapGradient);
}

template <>
void bindDerivedClass(py::class_<CustomStrip, Marker>& instance) {
    instance.def(py::init<transform::Position, std::string, size_t>());
    instance.def("setMarkerStripType", &CustomStrip::setMarkerStripType);
    instance.def("setRotations", &CustomStrip::setRotations);
    instance.def("setPoints", &CustomStrip::setPoints);
    instance.def("setComponentParameter", &CustomStrip::setComponentParameter);
}

template <>
void bindDerivedClass(py::class_<Info, Marker>& instance) {
    instance.def(py::init<std::string, size_t>());
    instance.def("getInfoText", &Info::getInfoText);
    instance.def("getInfoValue", &Info::getInfoValue);
    instance.def("set", py::overload_cast<const std::string&, int>(&Info::set));
    instance.def("set", py::overload_cast<const std::string&, double>(&Info::set));
    instance.def("set", py::overload_cast<const std::string&, std::string&>(&Info::set));
    instance.def("set", py::overload_cast<const std::string&, bool>(&Info::set));
}

template <>
void bindDerivedClass(py::class_<RobotInfo, Marker>& instance) {
    instance.def(py::init<RobotIdentifier>());
    instance.def("addParam", py::overload_cast<std::string, std::string>(&RobotInfo::addParam));
    instance.def("addParam", py::overload_cast<std::string, double>(&RobotInfo::addParam));
    instance.def("addParam", py::overload_cast<std::string, bool>(&RobotInfo::addParam));
    instance.def("addParam", py::overload_cast<std::string, int>(&RobotInfo::addParam));
    instance.def("setStatus", &RobotInfo::setStatus);
}

}  // namespace luhsoccer::python