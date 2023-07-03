include(FetchContent)
FetchContent_Declare(gtest
  QUIET
  URL https://github.com/google/googletest/archive/refs/tags/v1.13.0.tar.gz
)
# configure build of googletest
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(gtest)

add_executable(
    unit_tests
    template/unit_test.cpp
    time/unit_test.cpp
    transform/unit_test_circular_buffer.cpp
    transform/unit_test_transform.cpp
    marker_service/unit_test.cpp
    observer/unit_test_static_observer.cpp

    local_planner/component_params_unit_test.cpp

    local_planner_components/world_model_test_helper.hpp
    local_planner_components/world_model_test_helper.cpp
    local_planner_components/shapes/unit_test_point_shape.cpp
    local_planner_components/shapes/unit_test_circle_shape.cpp
    local_planner_components/shapes/unit_test_line_shape.cpp
    local_planner_components/shapes/unit_test_arc_shape.cpp
    local_planner_components/shapes/unit_test_compose_shape.cpp
    local_planner_components/shapes/unit_test_rectangle_shape.cpp
    local_planner_components/shapes/shape_test_helpers.cpp
    local_planner_components/steps/unit_test_wait_step.cpp
    local_planner_components/steps/unit_test_condition_step.cpp
)

target_link_libraries(unit_tests PRIVATE
  gtest_main
# To add a new module for testing add it to this list. WARNING: Only library targets can be linked out-of-the-box.
  time
  transform
  game_data_provider
  marker_service
  local_planner_components
  local_planner
  skills
)

# automatic discovery of unit tests
include(GoogleTest)
gtest_discover_tests(unit_tests PROPERTIES LABELS "unit" DISCOVERY_TIMEOUT 240)

if(ENABLE_TEST_COVERAGE)
  if(CMAKE_COMPILER_IS_GNUCXX)
    include(CodeCoverage)
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/html)
    set(GCOVR_ADDITIONAL_ARGS --html --html-details -o html/test_coverage.html --html-title "luhsoccer Baguette Test Coverage Report")
    setup_target_for_coverage_gcovr_xml(NAME test_coverage EXECUTABLE unit_tests --gtest_output=xml DEPENDENCIES unit_tests transform BASE_DIRECTORY "${PROJECT_SOURCE_DIR}/modules")
  else()
    message(FATAL_ERROR "Code converage can currently only enabled with g++ compiler.")
  endif()
endif()