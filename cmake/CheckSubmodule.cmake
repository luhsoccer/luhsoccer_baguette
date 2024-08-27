
function(check_submodule_git SUBMODULE_NAME COMMIT)
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${SUBMODULE_NAME}/.git")
        message(FATAL_ERROR "The submodule ${SUBMODULE_NAME} was not downloaded! Please update submodules with `git submodule update --init --recursive`")
    endif()

    # Find the git executable
    find_package(Git REQUIRED)

    if(Git_FOUND)
        execute_process(
            COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${SUBMODULE_NAME}
            OUTPUT_VARIABLE GIT_COMMIT
        )

        string(STRIP ${GIT_COMMIT} GIT_COMMIT)

        if(NOT "${GIT_COMMIT}" STREQUAL "${COMMIT}")
            message(FATAL_ERROR "The submodule ${SUBMODULE_NAME} is not at the correct commit! Please update submodules with `git pull --recurse-submodules`")
        else()
            message(STATUS "Submodule ${SUBMODULE_NAME} is at the correct commit: ${GIT_COMMIT}")
        endif()

    else()
        message(FATAL_ERROR "Git was not found on your system!")
    endif()

endfunction()