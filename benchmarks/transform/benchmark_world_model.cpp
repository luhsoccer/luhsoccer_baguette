#include <benchmark/benchmark.h>
#include "transform/world_model.hpp"

static void bm_push_single_transform(benchmark::State& state) {
    luhsoccer::transform::WorldModel wm;

    for(auto _  : state) {
        luhsoccer::transform::TransformWithVelocity tf;
        tf.header.child_frame = "child_frame";
        tf.header.parent_frame = "";
        tf.header.stamp = luhsoccer::time::now();
        tf.transform = Eigen::Translation2d(0, 0) * Eigen::Rotation2Dd(0);
        wm.pushTransform(tf);
    }
}

BENCHMARK(bm_push_single_transform);

static void bm_push_multiple_transform(benchmark::State& state) {
    luhsoccer::transform::WorldModel wm;

    int counter = 0;
    for(auto _  : state) {
        luhsoccer::transform::TransformWithVelocity tf;
        counter++;
        tf.header.child_frame = "child_frame_" + std::to_string(counter);
        tf.header.parent_frame = "";
        tf.header.stamp = luhsoccer::time::now();
        tf.transform = Eigen::Translation2d(0, 0) * Eigen::Rotation2Dd(0);
        wm.pushTransform(tf);
    }
}

BENCHMARK(bm_push_multiple_transform);