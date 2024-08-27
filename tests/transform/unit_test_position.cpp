#include <gtest/gtest.h>

#include "transform/position.hpp"

namespace luhsoccer::transform {

TEST(PositionTest, getString) {
    transform::Position empty_pos("");

    EXPECT_EQ(empty_pos.getString(), "(0.000, 0.000, 0.000°)@World");
}

}  // namespace luhsoccer::transform