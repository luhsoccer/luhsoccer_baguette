#include <gtest/gtest.h>

#include <time/time.hpp>
namespace luhsoccer::time {

TEST(DurationTest, Getter) {
    Duration d0(0.0);
    EXPECT_DOUBLE_EQ(d0.asSec(), 0.0);
    EXPECT_EQ(d0.asNSec(), 0);

    Duration d1(1.0);
    EXPECT_DOUBLE_EQ(d1.asSec(), 1.0);
    EXPECT_EQ(d1.asNSec(), 1e9);

    Duration d2(1.5);
    EXPECT_DOUBLE_EQ(d2.asSec(), 1.5);
    EXPECT_EQ(d2.asNSec(), 1.5e9);

    Duration d3(1000);
    EXPECT_DOUBLE_EQ(d3.asSec(), 1e-6);
    EXPECT_EQ(d3.asNSec(), 1000);

    Duration d4(-1.0);
    EXPECT_DOUBLE_EQ(d4.asSec(), -1.0);
    EXPECT_EQ(d4.asNSec(), -1e9);

    Duration d5(1, 500000000);
    EXPECT_DOUBLE_EQ(d5.asSec(), 1.5);
    EXPECT_EQ(d5.asNSec(), 1.5e9);
}

TEST(DurationTest, Arithmetic) {
    Duration d1(1.0);
    Duration d_2(-2.0);
    Duration d2(2.0);
    Duration d3(3.0);

    EXPECT_DOUBLE_EQ(Duration(d1 + d2).asSec(), 3.0);
    EXPECT_DOUBLE_EQ(Duration(d1 + d_2).asSec(), -1.0);

    EXPECT_EQ(Duration(d1 + d2), d3);
    EXPECT_EQ(d1 + d2, d3);

    EXPECT_DOUBLE_EQ(Duration(d1 - d3).asSec(), -2.0);
    EXPECT_DOUBLE_EQ(Duration(d1 - d_2).asSec(), 3.0);

    EXPECT_GT(d3, d2);
    EXPECT_GT(d2, d1);
    EXPECT_GT(d2, d_2);
    EXPECT_GT(d1, d_2);
    EXPECT_EQ(d1, d1);
}

TEST(TimePointTest, Getter) {
    TimePoint t0(0.0);
    EXPECT_DOUBLE_EQ(t0.asSec(), 0.0);
    EXPECT_EQ(t0.asNSec(), 0);

    TimePoint t1(1.0);
    EXPECT_DOUBLE_EQ(t1.asSec(), 1.0);
    EXPECT_EQ(t1.asNSec(), 1e9);

    TimePoint t2(1.5);
    EXPECT_DOUBLE_EQ(t2.asSec(), 1.5);
    EXPECT_EQ(t2.asNSec(), 1.5e9);

    TimePoint t4(-1.0);
    EXPECT_DOUBLE_EQ(t4.asSec(), -1.0);
    EXPECT_EQ(t4.asNSec(), -1e9);
}

TEST(TimePointTest, Arithmetic) {
    TimePoint t0(1.0);
    Duration d1(1.0);
    EXPECT_EQ(t0 + d1, TimePoint(2.0));
    EXPECT_EQ(t0 - d1, TimePoint(0.0));
}

TEST(Time, String) {
    EXPECT_EQ(std::to_string(Duration(1.0)), "1.000000000");
    EXPECT_EQ(std::to_string(Duration(-2.3)), "-2.300000000");
    EXPECT_EQ(std::to_string(TimePoint(1.0)), "1.000000000");
    EXPECT_EQ(std::to_string(TimePoint(-2.3)), "-2.300000000");
}

// TEST(Rate, Sleep) {
//     const double frequency = 100.0;
//     Rate rate(frequency);
//     double average = 0;
//     for (double delay = 0.0; delay < 1.0 / frequency; delay += 1.0 / frequency / 10.0) {
//         TimePoint t0 = now();
//         std::cout << delay << "\n";
//         std::this_thread::sleep_for(time::Duration(delay));
//         rate.sleep();
//         TimePoint t1 = now();
//         average += Duration(t1 - t0).asSec();
//     }
//     average /= 10;
//     EXPECT_GE(average, 0.5 / frequency);
//     EXPECT_LT(average, 1.5 / frequency);
// }

// TEST(Rate, Stopwatch) {
//     constexpr double freq_start = 10;
//     constexpr double freq_end = 100;
//     constexpr double steps = 10;

//     for (int freq = freq_start; freq <= freq_end; freq += (freq_end - freq_start) / steps) {
//         Rate r(freq);
//         LoopStopwatch s("UNIT_TEST", freq);
//         for (int i = 0; i < 10; i++) {
//             r.sleep();
//             s.tik();
//         }
//         EXPECT_GT(s.measuredFrequency(), freq * 0.5);
//         EXPECT_LT(s.measuredFrequency(), freq * 1.5);
//         s.printResult();
//     }
// }
}  // namespace luhsoccer::time