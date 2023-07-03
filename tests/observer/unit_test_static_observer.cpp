#include <gtest/gtest.h>

#include <memory>

#include "transform/transform.hpp"
#include "observer/static_observer.hpp"

TEST(StaticObserverTest, bestPassReceiver) {
    // create World Model
    // std::string global_frame = "global_frame";
    // luhsoccer::transform::WorldModel wm(global_frame);

    // luhsoccer::transform::BallInfo b;
    // b.time = luhsoccer::time::now();
    // b.state = luhsoccer::transform::BallState::ON_FIELD;
    // b.robot = std::nullopt;

    // wm.pushNewBallInfo(b);

    // auto w = std::make_shared<luhsoccer::transform::WorldModel>(wm);
    // auto result = luhsoccer::observer::calculation::calculateBallPosession(w);

    // EXPECT_FALSE(result.has_value());
}