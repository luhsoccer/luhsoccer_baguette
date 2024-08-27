
find_package(benchmark REQUIRED)

add_executable(
    benchmarks

    transform/benchmark_world_model.cpp
)

target_link_libraries(benchmarks PRIVATE
    benchmark::benchmark_main
    transform
)