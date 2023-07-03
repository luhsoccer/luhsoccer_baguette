#include <gtest/gtest.h>

#include "transform/circular_buffer.hpp"

TEST(CircularBufferTest, Size) {
    luhsoccer::CircularBuffer<int> b4(4);

    luhsoccer::CircularBuffer<int> b10000(10000);

    luhsoccer::CircularBuffer<int> b0(0);

    EXPECT_EQ(b4.size(), 0);
    EXPECT_EQ(b10000.size(), 0);
    EXPECT_EQ(b0.size(), 0);

    for (int i = 1; i < 10003; i++) {
        b4.push(0);
        b10000.push(0);
        b0.push(0);
        EXPECT_EQ(b4.size(), std::min(i, 4));
        EXPECT_EQ(b10000.size(), std::min(i, 10000));
        EXPECT_EQ(b0.size(), std::min(i, 1));
    }
}

TEST(CircularBufferTest, PushAndAt) {
    luhsoccer::CircularBuffer<int> b(4);
    b.push(0);
    b.push(1);
    b.push(2);

    EXPECT_EQ(b.at(0), 2);
    EXPECT_EQ(b.at(1), 1);
    EXPECT_EQ(b.at(2), 0);
    try {
        b.at(3);
        FAIL();
    } catch (std::out_of_range& e) {
    }
    try {
        b.at(4);

        FAIL();
    } catch (std::out_of_range& e) {
    }
}

TEST(CircularBufferTest, PreviousBufferSize) {
    auto b = std::make_shared<luhsoccer::CircularBuffer<int>>(4);
    EXPECT_EQ(b->size(), 0);
    for (int i = 0; i < 4; i++) {
        b->push(i);
    }
    EXPECT_EQ(b->size(), 4);
    luhsoccer::CircularBuffer<int> b2(4, b);
    // 4 elements from previous buffer
    EXPECT_EQ(b2.size(), 4);

    // increase when pushing to second buffer
    b2.push(0);
    EXPECT_EQ(b2.size(), 5);

    // decrease when pushing to previous buffer
    // prev buffer is in next loop
    for (int i = 0; i < 4; i++) {
        b->push(0);
        EXPECT_EQ(b2.size(), 4 - i);
    }

    // not decrease when old buffer is completely overwritten
    // prev buffer in more than in the next loop
    for (int i = 0; i < 4; i++) {
        b->push(0);
        EXPECT_EQ(b2.size(), 1);
    }

    // increase when pushing to second buffer
    for (int i = 0; i < 4; i++) {
        b2.push(0);
        EXPECT_EQ(b2.size(), std::min(2 + i, 4));
    }
}

TEST(CircularBufferTest, PreviousBufferSize2) {
    auto b = std::make_shared<luhsoccer::CircularBuffer<int>>(4);
    EXPECT_EQ(b->size(), 0);
    for (int i = 0; i < 2; i++) {
        b->push(i);
    }
    EXPECT_EQ(b->size(), 2);

    auto b2 = std::make_shared<luhsoccer::CircularBuffer<int>>(4, b);
    EXPECT_EQ(b2->size(), 2);

    b->push(0);
    // do not decrease because buffer is not looped around yet
    EXPECT_EQ(b2->size(), 2);

    b->push(0);
    // do not decrease because buffer is not looped around yet
    EXPECT_EQ(b2->size(), 2);

    for (int i = 0; i < 4; i++) {
        b2->push(i);
        EXPECT_EQ(b2->size(), 3 + i);
    }
    for (int i = 0; i < 4; i++) {
        b2->push(i);
        EXPECT_EQ(b2->size(), 4);
    }
}

TEST(CircularBufferTest, PreviousBufferAccess) {
    auto b = std::make_shared<luhsoccer::CircularBuffer<int>>(4);
    for (int i = 0; i < 2; i++) {
        b->push(i);
    }
    auto b2 = std::make_shared<luhsoccer::CircularBuffer<int>>(4, b);
    for (int i = 11; i < 15; i++) {
        b2->push(i);
        EXPECT_EQ(b2->at(0), i);
    }
    b->push(3);
    // b : | 00 | 01 | 03 |
    // b2:           | 11 | 12 | 13 | 14 |

    for (int i = 0; i < 4; i++) {
        EXPECT_EQ(b2->at(i), 14 - i);
    }
    for (int i = 0; i < 2; i++) {
        EXPECT_EQ(b->at(1 + i), 1 - i);
        EXPECT_EQ(b2->at(4 + i), 1 - i);
    }
    b2->push(15);
    try {
        // NOLINTNEXTLINE
        b2->at(6);
        FAIL();
    } catch (std::out_of_range& e) {
    }
    // b : | 00 | 01 | 03 |
    // b2:           | -- | 12 | 13 | 14 | 15 |
    for (int i = 0; i < 4; i++) {
        EXPECT_EQ(b2->at(i), 15 - i);
    }
    try {
        // NOLINTNEXTLINE
        b2->at(4);
        FAIL();
    } catch (std::out_of_range& e) {
    }

    auto b2_1 = std::make_shared<luhsoccer::CircularBuffer<int>>(4, b);
    auto b2_2 = std::make_shared<luhsoccer::CircularBuffer<int>>(4, b2_1);
    // b    : | 00 | 01 | 03 |
    // b2_1 :                | ...
    // b2_2 :                | ...
    EXPECT_EQ(b2_2->at(0), 3);
    EXPECT_EQ(b2_2->at(1), 1);
    EXPECT_EQ(b2_2->at(2), 0);
    b2_2->push(-1);
    // b    : | 00 | 01 | 03 |
    // b2_1 :                | ...
    // b2_2 :                | -1 |
    EXPECT_EQ(b2_2->at(0), -1);

    EXPECT_EQ(b2_2->at(1), 3);
    EXPECT_EQ(b2_2->at(2), 1);
    EXPECT_EQ(b2_2->at(3), 0);
    auto b2_3 = std::make_shared<luhsoccer::CircularBuffer<int>>(4, b2_2);
    // b    : | 00 | 01 | 03 |
    // b2_1 :                | ...
    // b2_2 :                | -1 |
    // b2_3 :                     | ...
    EXPECT_EQ(b2_3->at(0), -1);
    EXPECT_EQ(b2_3->at(1), 3);
    EXPECT_EQ(b2_3->at(2), 1);
    EXPECT_EQ(b2_3->at(3), 0);
}