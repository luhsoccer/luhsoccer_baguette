#include <gtest/gtest.h>

#include "transform/transform.hpp"

namespace luhsoccer::transform {
TEST(WorldModelTest, pushTransform) {
    std::string global_frame = "global_frame";
    std::string child_frame1 = "child_frame1";
    std::string child_frame2 = "child_frame2";

    WorldModel wm(global_frame);

    EXPECT_EQ(wm.getGlobalFrame(), global_frame);
    EXPECT_EQ(wm.getAllTransformFrames().size(), 1);
    EXPECT_EQ(wm.getAllTransformFrames().at(0), global_frame);

    // normal transform push
    TransformWithVelocity t;
    t.header.child_frame = child_frame1;
    t.header.parent_frame = global_frame;
    t.header.stamp = time::now();
    t.transform = Eigen::Translation2d(0, 0) * Eigen::Rotation2Dd(0);

    EXPECT_TRUE(wm.pushTransform(t, false));
    // same time stamp
    EXPECT_FALSE(wm.pushTransform(t, false));
    t.header.stamp = time::now();
    EXPECT_TRUE(wm.pushTransform(t, false));

    EXPECT_EQ(wm.getAllTransformFrames().size(), 2);
    EXPECT_EQ(wm.getTransform(child_frame1)->header.child_frame, child_frame1);

    // parent not known
    TransformWithVelocity t2;
    t2.header.child_frame = child_frame1;
    t2.header.parent_frame = "not known";
    t2.header.stamp = time::now();

    EXPECT_FALSE(wm.pushTransform(t2, false));

    // data to old
    TransformWithVelocity t3;
    t3.header.child_frame = child_frame1;
    t3.header.parent_frame = global_frame;
    t3.header.stamp = time::now() - time::Duration(1.0);

    EXPECT_FALSE(wm.pushTransform(t3, false));

    // parent not known
    TransformWithVelocity t4;
    t4.header.child_frame = child_frame1;
    t4.header.parent_frame = child_frame2;
    t4.header.stamp = time::now();

    EXPECT_FALSE(wm.pushTransform(t4, false));

    EXPECT_EQ(wm.getAllTransformFrames().size(), 2);
    EXPECT_EQ(wm.getTransform(child_frame1)->header.child_frame, child_frame1);

    // parent known but not global frame
    TransformWithVelocity t5;
    t5.header.child_frame = child_frame2;
    t5.header.parent_frame = child_frame1;
    t5.header.stamp = time::now();

    EXPECT_TRUE(wm.pushTransform(t5, false));

    EXPECT_EQ(wm.getAllTransformFrames().size(), 3);
    // This test can't be rewritten since get transform returns nullopt because no elements exists in the buffer
    //  EXPECT_EQ(wm.getAllTransformFrames().at(2), child_frame2);
}

TEST(WorldModelTest, getTransformStatic) {
    std::string global_frame = "global_frame";
    std::string child_frame1 = "child_frame1";
    std::string child_frame2 = "child_frame2";

    WorldModel wm(global_frame);

    TransformWithVelocity t;
    t.header.child_frame = child_frame1;
    t.header.parent_frame = global_frame;
    t.header.stamp = time::now();
    t.transform = Eigen::Translation2d(0, 0) * Eigen::Rotation2Dd(0);
    EXPECT_DOUBLE_EQ(t.transform->translation().x(), 0.0);
    EXPECT_DOUBLE_EQ(t.transform->translation().y(), 0.0);
    EXPECT_DOUBLE_EQ(Eigen::Rotation2Dd(t.transform->rotation()).angle(), 0.0);
    EXPECT_TRUE(wm.pushTransform(t, true));

    EXPECT_TRUE(wm.getTransform(child_frame1));
    EXPECT_TRUE(wm.getTransform(child_frame1, global_frame));
    EXPECT_TRUE(wm.getTransform(child_frame1, global_frame, time::now()));

    EXPECT_FALSE(wm.getTransform(child_frame2));

    // zero transform
    time::TimePoint time_point = time::now();
    auto t_r = wm.getTransform(global_frame, global_frame, time_point);

    EXPECT_TRUE(t_r);

    EXPECT_EQ(t_r->header.child_frame, global_frame);
    EXPECT_EQ(t_r->header.parent_frame, global_frame);
    EXPECT_EQ(t_r->header.stamp, time_point);
    EXPECT_DOUBLE_EQ(t_r->transform.translation().x(), 0.0);
    EXPECT_DOUBLE_EQ(t_r->transform.translation().y(), 0.0);
    EXPECT_DOUBLE_EQ(Eigen::Rotation2Dd(t_r->transform.rotation()).angle(), 0.0);

    t_r = wm.getTransform(child_frame1);

    EXPECT_TRUE(t_r);

    EXPECT_EQ(t_r->header.child_frame, child_frame1);
    EXPECT_EQ(t_r->header.parent_frame, global_frame);
    EXPECT_EQ(t_r->header.stamp, t.header.stamp);
    EXPECT_DOUBLE_EQ(t_r->transform.translation().x(), 0.0);
    EXPECT_DOUBLE_EQ(t_r->transform.translation().y(), 0.0);
    EXPECT_DOUBLE_EQ(Eigen::Rotation2Dd(t_r->transform.rotation()).angle(), 0.0);
}

TEST(WorldModelTest, getTransformDynamic) {
    std::string global_frame = "global_frame";
    std::string child_frame1 = "child_frame1";
    std::string child_frame2 = "child_frame2";

    WorldModel wm(global_frame);
    time::TimePoint start = time::now();

    for (int i = 0; i < 10; i++) {
        TransformWithVelocity t;
        t.header.child_frame = child_frame1;
        t.header.parent_frame = global_frame;
        t.header.stamp = start + i * time::Duration(0.01);
        t.transform = Eigen::Translation2d(i, 0.5 * i) * Eigen::Rotation2Dd(0.1 * i);
        EXPECT_TRUE(wm.pushTransform(t, false));
    }

    EXPECT_TRUE(wm.getTransform(child_frame1));
    EXPECT_TRUE(wm.getTransform(child_frame1, global_frame));

    // request with time to far in past
    EXPECT_FALSE(wm.getTransform(child_frame1, global_frame, start - time::Duration(0.1)));

    auto t1 = wm.getTransform(child_frame1, global_frame, start + time::Duration(0.01));
    EXPECT_TRUE(t1);
    EXPECT_DOUBLE_EQ(t1->transform.translation().x(), 1.0);
    EXPECT_DOUBLE_EQ(t1->transform.translation().y(), 0.5);
    EXPECT_DOUBLE_EQ(Eigen::Rotation2Dd(t1->transform.rotation()).angle(), 0.1);

    auto t2_5 = wm.getTransform(child_frame1, global_frame, start + time::Duration(0.025));
    EXPECT_TRUE(t2_5);
    EXPECT_DOUBLE_EQ(t2_5->transform.translation().x(), 2.5);
    EXPECT_DOUBLE_EQ(t2_5->transform.translation().y(), 1.25);
    EXPECT_DOUBLE_EQ(Eigen::Rotation2Dd(t2_5->transform.rotation()).angle(), 0.25);

    // extrapolate into future
    auto t15 = wm.getTransform(child_frame1, global_frame, start + time::Duration(0.15));
    EXPECT_TRUE(t15);
    EXPECT_DOUBLE_EQ(t15->transform.translation().x(), 15.0);
    EXPECT_DOUBLE_EQ(t15->transform.translation().y(), 7.5);
    EXPECT_DOUBLE_EQ(Eigen::Rotation2Dd(t15->transform.rotation()).angle(), 1.5);
}

TEST(WorldModelTest, getTransformInChild) {
    std::string global_frame = "global_frame";
    std::string child_frame1 = "child_frame1";
    std::string child_frame2 = "child_frame2";

    WorldModel wm(global_frame);
    time::TimePoint start = time::now();

    for (int i = 0; i < 10; i++) {
        TransformWithVelocity t;
        t.header.child_frame = child_frame1;
        t.header.parent_frame = global_frame;
        t.header.stamp = start + i * time::Duration(0.01);
        t.transform = Eigen::Translation2d(i, 0.5 * i) * Eigen::Rotation2Dd(0.1 * i);
        EXPECT_TRUE(wm.pushTransform(t, false));

        TransformWithVelocity t1;
        t1.header.child_frame = child_frame2;
        t1.header.parent_frame = global_frame;
        t1.header.stamp = start + i * time::Duration(0.01);
        t1.transform = Eigen::Translation2d(-i, -0.5 * i) * Eigen::Rotation2Dd(0);
        EXPECT_TRUE(wm.pushTransform(t1, false));
    }

    auto t2_5 = wm.getTransform(child_frame1, child_frame2, start + time::Duration(0.025));
    EXPECT_TRUE(t2_5);
    EXPECT_DOUBLE_EQ(t2_5->transform.translation().x(), 5.0);
    EXPECT_DOUBLE_EQ(t2_5->transform.translation().y(), 2.5);
    EXPECT_DOUBLE_EQ(Eigen::Rotation2Dd(t2_5->transform.rotation()).angle(), 0.25);

    // extrapolate into future
    auto t15 = wm.getTransform(child_frame1, child_frame2, start + time::Duration(0.15));
    EXPECT_TRUE(t15);
    EXPECT_DOUBLE_EQ(t15->transform.translation().x(), 30.0);
    EXPECT_DOUBLE_EQ(t15->transform.translation().y(), 15.0);
    EXPECT_DOUBLE_EQ(Eigen::Rotation2Dd(t15->transform.rotation()).angle(), 1.5);
}

TEST(WorldModelTest, Velocity) {
    std::string global_frame = "global_frame";
    std::string child_frame1 = "child_frame1";
    std::string child_frame2 = "child_frame2";

    WorldModel wm(global_frame);
    time::TimePoint start = time::now();

    for (int i = 0; i < 10; i++) {
        TransformWithVelocity t;
        t.header.child_frame = child_frame1;
        t.header.parent_frame = global_frame;
        constexpr double dt = 0.01;
        t.header.stamp = start + i * time::Duration(dt);
        t.transform = Eigen::Translation2d(5.0 * i * i * dt * dt, 2.5 * i * i * dt * dt) *
                      Eigen::Rotation2Dd(0.0 * i * i * dt * dt);
        t.velocity = Eigen::Vector3d(10.0 * i, 5.0 * i, 0.0 * i);
        EXPECT_TRUE(wm.pushTransform(t, false));

        TransformWithVelocity t1;
        t1.header.child_frame = child_frame2;
        t1.header.parent_frame = global_frame;
        t1.header.stamp = start + i * time::Duration(0.01);
        t1.transform =
            Eigen::Translation2d(-5.0 * i * i * dt * dt, 2.5 * i * i * dt * dt) * Eigen::Rotation2Dd(L_PI / 2);
        t1.velocity = Eigen::Vector3d(-10.0 * i, 5.0 * i, 0.0 * i);
        EXPECT_TRUE(wm.pushTransform(t1, false));
    }

    {
        auto tm = wm.getVelocity(child_frame1, "", "", start + time::Duration(0.09));
        EXPECT_TRUE(tm);
        EXPECT_DOUBLE_EQ(tm->velocity.x(), 90.0);
        EXPECT_DOUBLE_EQ(tm->velocity.y(), 45.0);
        EXPECT_DOUBLE_EQ(tm->velocity.z(), 0.0);
    }
    {
        auto tm = wm.getVelocity(child_frame1, "", "", start + time::Duration(0.035));
        EXPECT_TRUE(tm);
        EXPECT_DOUBLE_EQ(tm->velocity.x(), 35.0);
        EXPECT_DOUBLE_EQ(tm->velocity.y(), 17.5);
        EXPECT_DOUBLE_EQ(tm->velocity.z(), 0.0);
    }
    {
        auto tm = wm.getVelocity(child_frame1, "", "", start + time::Duration(0.15));
        EXPECT_TRUE(tm);
        EXPECT_DOUBLE_EQ(tm->velocity.x(), 150.0);
        EXPECT_DOUBLE_EQ(tm->velocity.y(), 75.0);
        EXPECT_DOUBLE_EQ(tm->velocity.z(), 0.0);
    }

    {
        auto tm = wm.getVelocity(child_frame1, child_frame2, global_frame, start + time::Duration(0.15));
        EXPECT_TRUE(tm);
        EXPECT_DOUBLE_EQ(tm->velocity.x(), 300.0);
        EXPECT_DOUBLE_EQ(tm->velocity.y(), 0.0);
        EXPECT_DOUBLE_EQ(tm->velocity.z(), 0.0);
    }
    {
        auto tm = wm.getVelocity(child_frame1, child_frame2, child_frame1, start + time::Duration(0.15));
        EXPECT_TRUE(tm);
        EXPECT_DOUBLE_EQ(tm->velocity.x(), 300.0);
        EXPECT_DOUBLE_EQ(tm->velocity.y(), 0.0);
        EXPECT_DOUBLE_EQ(tm->velocity.z(), 0.0);
    }
    {
        auto tm = wm.getVelocity(child_frame1, child_frame2, child_frame2, start + time::Duration(0.15));
        EXPECT_TRUE(tm);
        EXPECT_NEAR(tm->velocity.x(), 0.0, 1.0e-13);
        EXPECT_DOUBLE_EQ(tm->velocity.y(), -300.0);
        EXPECT_DOUBLE_EQ(tm->velocity.z(), 0.0);
    }
}

TEST(WorldModelTest, VelocityWithoutVelocityInput) {
    std::string global_frame = "global_frame";
    std::string child_frame1 = "child_frame1";
    std::string child_frame2 = "child_frame2";

    WorldModel wm(global_frame);
    time::TimePoint start = time::now();

    for (int i = 0; i < 10; i++) {
        TransformWithVelocity t;
        t.header.child_frame = child_frame1;
        t.header.parent_frame = global_frame;
        constexpr double dt = 0.01;
        t.header.stamp = start + i * time::Duration(dt);
        t.transform = Eigen::Translation2d(10.0 * i * dt, 5.0 * i * dt) * Eigen::Rotation2Dd(0.0);
        // t.velocity = Eigen::Vector3d(10.0 * i, 5.0 * i, 0.0 * i);
        EXPECT_TRUE(wm.pushTransform(t, false));

        TransformWithVelocity t1;
        t1.header.child_frame = child_frame2;
        t1.header.parent_frame = global_frame;
        t1.header.stamp = start + i * time::Duration(dt);
        t1.transform = Eigen::Translation2d(-10.0 * i * dt, 5.0 * i * dt) * Eigen::Rotation2Dd(L_PI / 2);
        // t1.velocity = Eigen::Vector3d(5.0 * i, 10.0 * i, 0.0 * i);
        EXPECT_TRUE(wm.pushTransform(t1, false));
    }

    {
        auto tm = wm.getVelocity(child_frame1, "", "", start + time::Duration(0.09));
        EXPECT_TRUE(tm);
        EXPECT_NEAR(tm->velocity.x(), 10.0, 1.0e-13);
        EXPECT_NEAR(tm->velocity.y(), 5.0, 1.0e-13);
        EXPECT_NEAR(tm->velocity.z(), 0.0, 1.0e-13);
    }
    {
        auto tm = wm.getVelocity(child_frame1, "", "", start + time::Duration(0.035));
        EXPECT_TRUE(tm);
        EXPECT_NEAR(tm->velocity.x(), 10.0, 1.0e-13);
        EXPECT_NEAR(tm->velocity.y(), 5.0, 1.0e-13);
        EXPECT_NEAR(tm->velocity.z(), 0.0, 1.0e-13);
    }
    {
        auto tm = wm.getVelocity(child_frame1, "", "", start + time::Duration(0.15));
        EXPECT_TRUE(tm);
        EXPECT_NEAR(tm->velocity.x(), 10.0, 1.0e-13);
        EXPECT_NEAR(tm->velocity.y(), 5.0, 1.0e-13);
        EXPECT_NEAR(tm->velocity.z(), 0.0, 1.0e-13);
    }
    {
        auto tm = wm.getVelocity(child_frame1, child_frame2, global_frame, start + time::Duration(0.15));
        EXPECT_TRUE(tm);
        EXPECT_NEAR(tm->velocity.x(), 20.0, 1.0e-13);
        EXPECT_NEAR(tm->velocity.y(), 0.0, 1.0e-13);
        EXPECT_NEAR(tm->velocity.z(), 0.0, 1.0e-13);
    }
    {
        auto tm = wm.getVelocity(child_frame1, child_frame2, child_frame1, start + time::Duration(0.15));
        EXPECT_TRUE(tm);
        EXPECT_NEAR(tm->velocity.x(), 20.0, 1.0e-13);
        EXPECT_NEAR(tm->velocity.y(), 0.0, 1.0e-13);
        EXPECT_NEAR(tm->velocity.z(), 0.0, 1.0e-13);
    }
    {
        auto tm = wm.getVelocity(child_frame1, child_frame2, child_frame2, start + time::Duration(0.15));
        EXPECT_TRUE(tm);
        EXPECT_NEAR(tm->velocity.x(), 0.0, 1.0e-13);
        EXPECT_NEAR(tm->velocity.y(), -20.0, 1.0e-13);
        EXPECT_NEAR(tm->velocity.z(), 0.0, 1.0e-13);
    }
}

TEST(PositionTest, General) {
    std::string global_frame = "global_frame";
    std::string child_frame1 = "child_frame1";
    std::string child_frame2 = "child_frame2";

    auto wm = std::make_shared<WorldModel>(global_frame);
    time::TimePoint start = time::now();

    for (int i = 0; i < 10; i++) {
        TransformWithVelocity t;
        t.header.child_frame = child_frame1;
        t.header.parent_frame = global_frame;
        t.header.stamp = start + i * time::Duration(0.01);
        t.transform = Eigen::Translation2d(i, 0.5 * i) * Eigen::Rotation2Dd(0.0 * i);
        EXPECT_TRUE(wm->pushTransform(t, false));

        TransformWithVelocity t1;
        t1.header.child_frame = child_frame2;
        t1.header.parent_frame = global_frame;
        t1.header.stamp = start + i * time::Duration(0.01);
        t1.transform = Eigen::Translation2d(-i, -0.5 * i) * Eigen::Rotation2Dd(0);
        EXPECT_TRUE(wm->pushTransform(t1, false));
    }

    Position p1(child_frame1, Eigen::Translation2d(1, 2) * Eigen::Rotation2Dd(1.0));
    EXPECT_EQ(p1.getFrame(), child_frame1);
    {
        auto tl = p1.getCurrentPosition(wm, global_frame);
        EXPECT_TRUE(tl);
        EXPECT_DOUBLE_EQ(tl->translation().x(), 10.0);
        EXPECT_DOUBLE_EQ(tl->translation().y(), 6.5);
        EXPECT_DOUBLE_EQ(Eigen::Rotation2Dd(tl->rotation()).angle(), 1.0);
    }
    {
        auto tl = p1.getCurrentPosition(wm, global_frame, start + time::Duration(0.035));
        EXPECT_TRUE(tl);
        EXPECT_DOUBLE_EQ(tl->translation().x(), 4.5);
        EXPECT_DOUBLE_EQ(tl->translation().y(), 3.75);
        EXPECT_DOUBLE_EQ(Eigen::Rotation2Dd(tl->rotation()).angle(), 1.0);
    }
}

TEST(PositionTest, RelativeToPosition) {
    std::string global_frame = "global_frame";
    std::string child_frame1 = "child_frame1";
    std::string child_frame2 = "child_frame2";

    auto wm = std::make_shared<WorldModel>(global_frame);
    time::TimePoint start = time::now();

    TransformWithVelocity t;
    t.header.child_frame = child_frame1;
    t.header.parent_frame = global_frame;
    t.header.stamp = start;
    t.transform = Eigen::Translation2d(1, 1) * Eigen::Rotation2Dd(0.0);
    EXPECT_TRUE(wm->pushTransform(t, true));

    TransformWithVelocity t1;
    t1.header.child_frame = child_frame2;
    t1.header.parent_frame = global_frame;
    t1.header.stamp = start;
    t1.transform = Eigen::Translation2d(-1, 1) * Eigen::Rotation2Dd(L_PI / 2.0);
    EXPECT_TRUE(wm->pushTransform(t1, true));

    Position p1(child_frame1, 1, 1, L_PI / 2.0);
    Position p2(child_frame2, 1, 1, -L_PI / 2.0);

    auto t_res = p2.getCurrentPosition(wm, p1, start);

    ASSERT_TRUE(t_res);

    EXPECT_NEAR(t_res->translation().x(), 0.0, 1.0e-10);
    EXPECT_DOUBLE_EQ(t_res->translation().y(), 4.0);
    EXPECT_DOUBLE_EQ(Eigen::Rotation2Dd(t_res->rotation()).angle(), -L_PI / 2.0);
}

}  // namespace luhsoccer::transform