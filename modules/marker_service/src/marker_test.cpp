#include "marker_service/marker_test.hpp"
#include "marker_service/marker.hpp"

namespace luhsoccer::marker {

int MarkerTest::random(int min, int max) { return rand() % max + min; }

void MarkerTest::displayTestMarkers() {
    auto wm = gdp.getWorldModel();

    std::string global_frame = wm->getGlobalFrame();

    // rect
    marker::Rect rect(global_frame, "rect", 1);
    rect.setSize({1, 1});
    rect.setThickness(random(1, 3) / 100.0f);
    rect.setColor(marker::Color::YELLOW());
    ms.displayMarker(rect);

    // rect filled
    marker::Rect rect_filled(global_frame, "rect", 2);
    rect_filled.setSize({9, 6});
    rect_filled.setFilled(true);
    rect_filled.setHeight(-0.01);
    rect_filled.setColor({marker::Color::hsv2Rgb(120, 100, 60, 1)});
    rect_filled.setLifetime(5);
    ms.displayMarker(rect_filled);

    // heatmap
    marker::Heatmap heatmap({global_frame, -3, 1.5, 0}, "heatmap", 1);
    heatmap.setHeatmapPoints(
        Eigen::MatrixXd{{3.6, 7.59, 1.19, 5.94, 7.39, 2.61, 9.42, 6.85, 4.35, 8.68},
                        {9.98, 1.47, static_cast<double>(random(2, 7)), 6.03, 8.63, 8.97, 1.17, 9.31, 6.18, 4.6},
                        {8.21, 5.87, 2.33, 9.76, 6.81, 7.41, 7.79, 3.59, 3.31, 3.01},
                        {5.73, 6.3, 9.74, 7.11, 6.1, 5.45, 3.92, 7.15, 7.92, 1.12},
                        {9.98, 1.47, 4.64, 6.03, 8.63, 8.97, 1.17, 9.31, 6.18, 4.6},
                        {8.21, 5.87, 2.33, 9.76, 6.81, 7.41, 7.79, 3.59, 3.31, 3.01},
                        {4.37, 3.62, 5.26, 1.11, 7.59, 6.54, 8.63, 7.35, 5.77, 6.6},
                        {8.21, 5.87, 2.33, 9.76, 6.81, 7.41, 7.79, 3.59, 3.31, 3.01},
                        {9.98, 1.47, 4.64, 6.03, 8.63, 8.97, 1.17, 9.31, 6.18, 4.6},
                        {5.73, 6.3, 9.74, 7.11, 6.1, 5.45, 3.92, 7.15, 7.92, 1.12}});
    heatmap.setHeatmapColors({Color::BLUE(), Color::RED()});
    heatmap.setHeatmapSize({2, 2});
    ms.displayMarker(heatmap);

    // Circular Heatmap
    marker::CircularHeatmap c_heatmap({global_frame, 0, 2, 0}, "heatmap", 2);
    c_heatmap.setCircularHeatmapColors({Color::RED(), Color::BLUE()});
    c_heatmap.setCircularHeatmapData({2, 4,  5,  6,  1,  5,  8,  9,  4,  1,  10, 2,  3, 4, 2, 1, 2, 3, 4, 5, 6, 7, 8,
                                      9, 10, 11, 12, 13, 14, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0});
    c_heatmap.setRadius(0.5);
    ms.displayMarker(c_heatmap);

    // Circular Heatmap with gradient
    marker::CircularHeatmap c_heatmap2({global_frame, 3, 0.7, 0}, "heatmap", 3);
    c_heatmap2.setHeatmapGradient({marker::Color::RED(), Color::GREEN(), Color::BLUE(), Color::ORANGE()});
    c_heatmap2.setCircularHeatmapData({2, 4,  5,  6,  1,  5,  8,  9,  4,  1,  10, 2,  3, 4, 2, 1, 2, 3, 4, 5, 6, 7, 8,
                                       9, 10, 11, 12, 13, 14, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0});
    c_heatmap2.setRadius(0.7);
    ms.displayMarker(c_heatmap2);

    // custom strip 2d arrows
    marker::CustomStrip customStrip({global_frame, -1, 1, 0}, "customStrip", 1);
    customStrip.setMarkerStripType(MType::ARROW_2D);
    customStrip.setPoints({{-0.2, 0.1}, {-0.2, 0.3}, {-0.3, 0.4}});
    customStrip.setRotations({1.5, 0.8, 0.5});
    customStrip.setComponentParameter(SizeVec2{0.1, 0.05});
    customStrip.setColor(Color::RED());
    ms.displayMarker(customStrip);

    // custom strip rects
    marker::CustomStrip customStrip2({global_frame, -1, 0.5, 0}, "customStrip", 2);
    customStrip2.setMarkerStripType(MType::RECT);
    customStrip2.setPoints({{-0.2, 0.1}, {-0.2, 0.3}, {-0.3, 0.4}});
    customStrip2.setRotations({1.5, 0.8, 0.5});
    customStrip2.setComponentParameter(SizeVec2{0.1, 0.05});
    customStrip2.setColor(Color::RED());
    ms.displayMarker(customStrip2);

    std::vector<Point> points{};
    static int size = 1;
    int id = 1;
    for (int i = 0; i < size; ++i) {
        points.emplace_back(Point{static_cast<double>(random(-5, 5)), static_cast<double>(random(-5, 5))});
        marker::LineStrip marker(global_frame, "experiment_log_test", id);
        marker.setColor(marker::Color::WHITE());
        marker.setPathClosed(false);
        marker.setPoints(points);
        this->ms.displayMarker(marker);
    }
    ++size;

    // linestrip
    marker::LineStrip linestrip({global_frame, 0, 0, 0}, "line", 1);
    linestrip.setColors({marker::Color::RED(), marker::Color::BLACK(), marker::Color::BLUE(), marker::Color::YELLOW()});
    linestrip.setPoints({{0, 0}, {0.5, 0.5}, {1, 0.5}, {1, 1}});
    linestrip.setPathClosed(false);
    ms.displayMarker(linestrip);

    // line
    marker::Line line(global_frame, "line", 2);
    line.setLinePoints({1, 1}, {2, 2});
    line.setThickness(0.1);
    line.setColor(marker::Color::LIGHT_GREEN());
    ms.displayMarker(line);

    // Text
    marker::Text text({global_frame, 2, -1, 0}, "text", 1);
    text.setText("text sample 1234 !?");
    text.setColor(Color::WHITE());
    ms.displayMarker(text);

    // circle
    marker::Circle c({global_frame, 0, -0.8}, "circle", 1);
    c.setRadius(0.25);
    c.setThickness(0.03);
    c.setColor(marker::Color::ORANGE());
    ms.displayMarker(c);

    // circle filled
    marker::Circle c2({global_frame, -2, -0.8}, "circle", 2);
    c2.setRadius(0.5);
    c2.setColor(marker::Color::LIGHT_BLUE());
    c2.setFilled(true);
    ms.displayMarker(c2);

    // Cube
    double y = -2;
    double x = -3.5;
    double add = 0.8;
    double height = 0.2;
    marker::Cube cube({global_frame, x, y, 0}, "models", 1);
    cube.setHeight(height);
    cube.setColor(Color::random());
    ms.displayMarker(cube);
    x += add;

    // Robot
    marker::Robot robot({global_frame, x, y, 0}, gdp.getGoalie(), "models", 2);
    robot.setColor(Color::random());
    robot.setHeight(height);
    ms.displayMarker(robot);
    x += add;

    // GoalBorder
    marker::GoalBorder goal_border({global_frame, x, y, 0}, "models", 3);
    goal_border.setColor(Color::random());
    goal_border.setHeight(height);
    ms.displayMarker(goal_border);
    x += add;

    // Ball
    marker::Ball ball({global_frame, x, y, 0}, "models", 4);
    ball.setColor(Color::random());
    ball.setHeight(height);
    ms.displayMarker(ball);
    x += add;

    // Cone
    marker::Cone cone({global_frame, x, y, 0}, "models", 5);
    cone.setColor(Color::random());
    cone.setHeight(height);
    ms.displayMarker(cone);
    x += add;

    // Cylinder
    marker::Cylinder cylinder({global_frame, x, y, 0}, "models", 6);
    cylinder.setColor(Color::random());
    cylinder.setHeight(height);
    ms.displayMarker(cylinder);
    x += add;

    // Sphere
    marker::Sphere sphere({global_frame, x, y, 0}, "models", 7);
    sphere.setColor(Color::random());
    sphere.setHeight(height);
    ms.displayMarker(sphere);
    x += add;

    // Torus
    marker::Torus torus({global_frame, x, y, 0}, "models", 8);
    torus.setColor(Color::random());
    torus.setHeight(height);
    ms.displayMarker(torus);
    x += add;

    // Arrow
    marker::Arrow arrow({global_frame, x, y, 0}, "models", 9);
    arrow.setColor(Color::random());
    arrow.setHeight(height);
    ms.displayMarker(arrow);
    x += add;

    // Suzanne
    marker::Suzanne suzanne({global_frame, x, y, 0}, "models", 10);
    suzanne.setColor(Color::random());
    suzanne.setHeight(height);
    suzanne.setScale(0.1);
    ms.displayMarker(suzanne);
}

}  // namespace luhsoccer::marker