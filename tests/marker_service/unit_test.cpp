#include <gtest/gtest.h>

#include <marker_service/marker_service.hpp>
namespace luhsoccer::marker {

transform::Position setup(std::string child_frame, double x, double y, double angle) {
    // create transform
    transform::WorldModel wm{"global_frame"};
    transform::TransformWithVelocity t1;
    t1.header.child_frame = child_frame;
    t1.header.parent_frame = "global_frame";
    t1.header.stamp = time::now();
    t1.transform = Eigen::Translation2d(0, 0) * Eigen::Rotation2Dd(0.0);
    wm.pushTransform(t1, false);
    wm.pushTransform(t1, false);
    return transform::Position{child_frame, Eigen::Translation2d(x, y) * Eigen::Rotation2Dd(angle)};
}

TEST(Marker, Marker3DPropertiesTest) {
    transform::Position p = setup("child_frame", 0, 0, 0);
    Sphere s{p};
    s.setColor({1, 0, 0, 0});
    ASSERT_DOUBLE_EQ(s.getColor().red, 1);
    ASSERT_DOUBLE_EQ(s.getColor().green, 0);
    ASSERT_DOUBLE_EQ(s.getColor().blue, 0);
    ASSERT_DOUBLE_EQ(s.getColor().alpha, 0);

    s.setFrameLocked(true);
    ASSERT_TRUE(s.isFrameLocked());
    s.setHeight(0.5);
    ASSERT_DOUBLE_EQ(s.getHeight(), 0.5);
    s.setId(1);
    ASSERT_EQ(s.getId(), 1);
    s.setLifetime(5.0);
    ASSERT_DOUBLE_EQ(s.getLifetime(), 5.0);
    s.setNs("test");
    ASSERT_STREQ(s.getNs().c_str(), "test");
    s.setScale({1, 2, 3});
    ASSERT_DOUBLE_EQ(s.getScale().x, 1);
    ASSERT_DOUBLE_EQ(s.getScale().y, 2);
    ASSERT_DOUBLE_EQ(s.getScale().z, 3);
}

TEST(Marker2D, TextPropertiesTest) {
    transform::Position p = setup("child_frame", 0, 0, 0);
    Text t{p};
    t.setText("Test text");
    ASSERT_STREQ(t.getText().c_str(), "Test text");
    // other props already tested in Marker3DPropertiesTest
}

TEST(Marker2D, 2DMarkerAndCirclePropertiesTest) {
    transform::Position p = setup("child_frame", 0, 0, 0);
    Circle c{p};
    c.setRadius(0.3);
    ASSERT_DOUBLE_EQ(c.getCircleRadius(), 0.3);
    c.setFilled(true);
    ASSERT_TRUE(c.isCircleFilled());
    c.setColor(Color::BLUE());
    ASSERT_DOUBLE_EQ(c.getColor().red, Color::BLUE().red);
    ASSERT_DOUBLE_EQ(c.getColor().green, Color::BLUE().green);
    ASSERT_DOUBLE_EQ(c.getColor().blue, Color::BLUE().blue);
    ASSERT_DOUBLE_EQ(c.getColor().alpha, Color::BLUE().alpha);
    c.setColors({Color::RED(), Color::BLUE()});
    ASSERT_EQ(c.getColors().size(), 2);
    ASSERT_DOUBLE_EQ(c.getColors()[0].red, Color::RED().red);
    ASSERT_DOUBLE_EQ(c.getColors()[0].green, Color::RED().green);
    ASSERT_DOUBLE_EQ(c.getColors()[0].blue, Color::RED().blue);
    ASSERT_DOUBLE_EQ(c.getColors()[0].alpha, Color::RED().alpha);
    ASSERT_DOUBLE_EQ(c.getColors()[1].red, Color::BLUE().red);
    ASSERT_DOUBLE_EQ(c.getColors()[1].green, Color::BLUE().green);
    ASSERT_DOUBLE_EQ(c.getColors()[1].blue, Color::BLUE().blue);
    ASSERT_DOUBLE_EQ(c.getColors()[1].alpha, Color::BLUE().alpha);
    c.setThickness(0.5);
    ASSERT_DOUBLE_EQ(c.getThickness(), 0.5);
}

TEST(Marker2D, RectPropertiesTest) {
    transform::Position p = setup("child_frame", 0, 0, 0);
    Rect r{p};
    r.setFilled(true);
    ASSERT_TRUE(r.isRectFilled());
    r.setSize({1, 0.5});
    ASSERT_DOUBLE_EQ(r.getRectSize().size_x, 1);
    ASSERT_DOUBLE_EQ(r.getRectSize().size_y, 0.5);
}

TEST(Marker2D, LinePropertiesTest) {
    transform::Position p = setup("child_frame", 0, 0, 0);
    Line l{p};
    l.setLinePoints({1, 0}, {0.4, 3, 0.3});
    ASSERT_DOUBLE_EQ(l.getLineStart().x, 1);
    ASSERT_DOUBLE_EQ(l.getLineStart().y, 0);
    ASSERT_DOUBLE_EQ(l.getLineEnd().x, 0.4);
    ASSERT_DOUBLE_EQ(l.getLineEnd().y, 3);
    ASSERT_DOUBLE_EQ(l.getLineEnd().z, 0.3);
}

TEST(Marker2D, ArrowPropertiesTest) {
    transform::Position p = setup("child_frame", 0, 0, 0);
    Arrow2d a{p};
    a.setSize({0.4, 2});
    ASSERT_DOUBLE_EQ(a.getArrowSize().size_x, 0.4);
    ASSERT_DOUBLE_EQ(a.getArrowSize().size_y, 2);
}

TEST(Marker2D, LinestripPropertiesTest) {
    transform::Position p = setup("child_frame", 0, 0, 0);
    LineStrip l{p};
    l.setPathClosed(true);
    ASSERT_TRUE(l.isLinestripClosed());
    l.setPoints({{0, 1}, {1, 2, 3}, {2.2, 3.7}});
    ASSERT_EQ(l.getPoints().size(), 3);
    ASSERT_DOUBLE_EQ(l.getPoints()[0].x, 0);
    ASSERT_DOUBLE_EQ(l.getPoints()[0].y, 1);
    ASSERT_DOUBLE_EQ(l.getPoints()[1].x, 1);
    ASSERT_DOUBLE_EQ(l.getPoints()[1].y, 2);
    ASSERT_DOUBLE_EQ(l.getPoints()[1].z, 3);
    ASSERT_DOUBLE_EQ(l.getPoints()[2].x, 2.2);
    ASSERT_DOUBLE_EQ(l.getPoints()[2].y, 3.7);
}

TEST(Marker2D, CustomStripPropertiesTest) {
    transform::Position p = setup("child_frame", 0, 0, 0);
    CustomStrip l{p};
    l.setMarkerStripType(MType::ARROW_2D);
    ASSERT_EQ(l.getMarkerStripType(), MType::ARROW_2D);
    l.setPoints({{0, 0.5}, {4, 2, 3}, {2.2, 3}});
    ASSERT_EQ(l.getPoints().size(), 3);
    ASSERT_DOUBLE_EQ(l.getPoints()[0].x, 0);
    ASSERT_DOUBLE_EQ(l.getPoints()[0].y, 0.5);
    ASSERT_DOUBLE_EQ(l.getPoints()[1].x, 4);
    ASSERT_DOUBLE_EQ(l.getPoints()[1].y, 2);
    ASSERT_DOUBLE_EQ(l.getPoints()[1].z, 3);
    ASSERT_DOUBLE_EQ(l.getPoints()[2].x, 2.2);
    ASSERT_DOUBLE_EQ(l.getPoints()[2].y, 3);
    l.setComponentParameter(SizeVec2{1, 2});
    ASSERT_DOUBLE_EQ(l.getArrowSize().size_x, 1);
    ASSERT_DOUBLE_EQ(l.getArrowSize().size_y, 2);
    ASSERT_DOUBLE_EQ(l.getRectSize().size_x, 1);
    ASSERT_DOUBLE_EQ(l.getRectSize().size_y, 2);
    l.setComponentParameter(0.1);
    ASSERT_DOUBLE_EQ(l.getCircleRadius(), 0.1);
}

TEST(Marker2D, heatmapPropertiesTest) {
    transform::Position p = setup("child_frame", 0, 0, 0);
    Heatmap l{p};
    l.setHeatmapColors({Color::RED(), Color::BLACK()});
    ASSERT_DOUBLE_EQ(l.getHeatmapColors()[0].red, Color::RED().red);
    ASSERT_DOUBLE_EQ(l.getHeatmapColors()[0].green, Color::RED().green);
    ASSERT_DOUBLE_EQ(l.getHeatmapColors()[0].blue, Color::RED().blue);
    ASSERT_DOUBLE_EQ(l.getHeatmapColors()[0].alpha, Color::RED().alpha);
    ASSERT_DOUBLE_EQ(l.getHeatmapColors()[1].red, Color::BLACK().red);
    ASSERT_DOUBLE_EQ(l.getHeatmapColors()[1].green, Color::BLACK().green);
    ASSERT_DOUBLE_EQ(l.getHeatmapColors()[1].blue, Color::BLACK().blue);
    ASSERT_DOUBLE_EQ(l.getHeatmapColors()[1].alpha, Color::BLACK().alpha);
    l.setHeatmapPoints(Eigen::MatrixXd{{1, 2}, {2.34, 4.5}});
    ASSERT_DOUBLE_EQ(l.getHeatmapData()(0, 0), 1);
    ASSERT_DOUBLE_EQ(l.getHeatmapData()(0, 1), 2);
    ASSERT_DOUBLE_EQ(l.getHeatmapData()(1, 0), 2.34);
    ASSERT_DOUBLE_EQ(l.getHeatmapData()(1, 1), 4.5);
}

TEST(MarkerSerivce, MarkerServiceText) {
    // auto wm = std::make_shared<transform::WorldModel>("global_frame2");
    // transform::TransformWithVelocity t1;
    // t1.header.child_frame = "child_frame2";
    // t1.header.parent_frame = "global_frame2";
    // t1.header.stamp = time::now();
    // t1.transform = Eigen::Translation2d(0, 0) * Eigen::Rotation2Dd(0.0);
    // wm->pushTransform(t1, true);
    // wm->pushTransform(t1, true);

    // transform::Position p{"child_frame2", Eigen::Translation2d(0, 0) * Eigen::Rotation2Dd(0)};
    // Arrow2d a{p, "test2", 5};

    // MarkerService ms{};
    // ms.setRealWorldmodel(wm);

    // LuhvizMarkers luhvizMarkers = ms.updateTransforms();
    // ASSERT_EQ(luhvizMarkers.markers2d.size(), 0);
    // ASSERT_EQ(luhvizMarkers.markers.size(), 2);  // +2 because the frame of the wm
    // ms.displayMarker(a);
    // luhvizMarkers = ms.updateTransforms();
    // ASSERT_EQ(luhvizMarkers.markers2d.size(), 1);
    // ASSERT_EQ(luhvizMarkers.markers.size(), 2);  // +2 because the frame of the wm
    // Sphere s{p};
    // ms.displayMarker(s);
    // luhvizMarkers = ms.updateTransforms();
    // ASSERT_EQ(luhvizMarkers.markers2d.size(), 1);
    // ASSERT_EQ(luhvizMarkers.markers.size(), 3);  // +2 because the frame of the wm
    // ms.deleteMarker(s.getNs(), s.getId());
    // luhvizMarkers = ms.updateTransforms();
    // ASSERT_EQ(luhvizMarkers.markers2d.size(), 1);
    // ASSERT_EQ(luhvizMarkers.markers.size(), 2);  // +2 because the frame of the wm
    // ms.deleteMarker(s.getNs(), s.getId());
    // luhvizMarkers = ms.updateTransforms();
    // ASSERT_EQ(luhvizMarkers.markers2d.size(), 1);
    // ASSERT_EQ(luhvizMarkers.markers.size(), 2);  // +2 because the frame of the wm
}

}  // namespace luhsoccer::marker