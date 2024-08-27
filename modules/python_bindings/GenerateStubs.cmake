find_program(PYTHON python)


if(NOT PYTHON)
    message(FATAL_ERROR "error: Can't generate stubs without python.\n")
endif()


set(WORKING_DIRECTORY $ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/baguette_py)

# Set the PYTHONPATH to the install directory. This is needed for the stubgen to find the baguette_py module.
# Windows and Unix use different separators.
if(UNIX)
    set(ENV{PYTHONPATH} "${WORKING_DIRECTORY}:$ENV{PYTHONPATH}")
else()
    set(ENV{PYTHONPATH} "${WORKING_DIRECTORY};$ENV{PYTHONPATH}")
endif()

set(ENV{ASAN_OPTIONS} "verify_asan_link_order=0")

message(STATUS "Environment variable PYTHONPATH: $ENV{PYTHONPATH}")
message(STATUS "Stubgen source: ${STUBGEN_SOURCE}")
# Generate the stubs in the install directory.
execute_process(
    COMMAND ${PYTHON} "${STUBGEN_SOURCE}"
    WORKING_DIRECTORY ${WORKING_DIRECTORY}
    RESULT_VARIABLE STUBGEN_RESULT
)

# Fail if the stub generation failed.
if(NOT STUBGEN_RESULT EQUAL 0)
    message(FATAL_ERROR "error: python stubgen.py failed with code ${STUBGEN_RESULT}")
else()
    message(STATUS "Successfully generated stubs.")
endif()