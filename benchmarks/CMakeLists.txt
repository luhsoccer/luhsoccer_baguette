include(FetchContent)
FetchContent_Declare(benchmark
    QUIET
    URL https://github.com/google/benchmark/archive/refs/tags/v1.7.1.tar.gz
)
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
# configure build of googletest
FetchContent_MakeAvailable(benchmark)

add_executable(
    benchmarks

    transform/benchmark_world_model.cpp
)

target_link_libraries(benchmarks PRIVATE
    benchmark_main
    transform
)