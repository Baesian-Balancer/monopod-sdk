# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

include(FetchContent)

# ===
# YCM
# ===

find_package(YCM QUIET)

option(SCENARIO_USE_SYSTEM_YCM
    "Use system-installed YCM, rather than a private copy"
    ${YCM_FOUND})

if(SCENARIO_USE_SYSTEM_YCM AND NOT ${YCM_FOUND})
    message(FATAL_ERROR "Failed to find system YCM")
endif()

if(NOT ${SCENARIO_USE_SYSTEM_YCM})

    FetchContent_Declare(
        ycm
        GIT_REPOSITORY https://github.com/robotology/ycm.git)

    FetchContent_GetProperties(ycm)

    if(NOT ycm_POPULATED)
        FetchContent_Populate(ycm)

        add_subdirectory(${ycm_SOURCE_DIR}
                         ${ycm_BINARY_DIR}
                         EXCLUDE_FROM_ALL)
    endif()

    set(CMAKE_MODULE_PATH
        "${CMAKE_MODULE_PATH};${ycm_SOURCE_DIR}/modules" PARENT_SCOPE)
else()
    set(CMAKE_MODULE_PATH
        "${CMAKE_MODULE_PATH};${YCM_MODULE_PATH}" PARENT_SCOPE)
endif()

# =============================================================================

function (build_external_project target target_DIR git_repo)

    # set(CMAKELIST_CONTENT "
    #     cmake_minimum_required(VERSION ${CMAKE_MINIMUM_REQUIRED_VERSION})
    #     project(build_external_project)
    #     include(ExternalProject)
    #     ExternalProject_add(${target}
    #         GIT_REPOSITORY \"${git_repo}\"
    #         CMAKE_GENERATOR \"${CMAKE_GENERATOR}\"
    #         CMAKE_GENERATOR_PLATFORM \"${CMAKE_GENERATOR_PLATFORM}\"
    #         CMAKE_GENERATOR_TOOLSET \"${CMAKE_GENERATOR_TOOLSET}\"
    #         CMAKE_GENERATOR_INSTANCE \"${CMAKE_GENERATOR_INSTANCE}\"
    #         CMAKE_ARGS ${ARGN})
    #     add_custom_target(build_external_project)
    #     add_dependencies(build_external_project ${target})
    # ")

    set(TARGET_DIR "${CMAKE_BINARY_DIR}/_deps/${target}")

    set(CMAKELIST_CONTENT "
        cmake_minimum_required(VERSION ${CMAKE_MINIMUM_REQUIRED_VERSION})
        project(build_external_project)
        include(ExternalProject)
        ExternalProject_add(${target}
            GIT_REPOSITORY \"${git_repo}\"
            # #--Configure step-------------
            # CONFIGURE_COMMAND cmake ${TARGET_DIR}
            # #--Build step-----------------
            # BUILD_COMMAND cmake --build ${TARGET_DIR}
            # #--Install step---------------
            # UPDATE_COMMAND \"\" # Skip annoying updates for every build
            # INSTALL_COMMAND \"\"

            # CONFIGURE_COMMAND \"\" # Disable all other steps
            # BUILD_COMMAND \"\"
            INSTALL_COMMAND \"\"
            CMAKE_ARGS ${ARGN})
        add_custom_target(build_external_project)
        add_dependencies(build_external_project ${target})
    ")

    # cmake -S . -B build

    file(WRITE "${TARGET_DIR}/CMakeLists.txt" "${CMAKELIST_CONTENT}")

    # file(MAKE_DIRECTORY "${TARGET_DIR}" "${TARGET_DIR}/build")

    # # execute_process(COMMAND ${CMAKE_COMMAND}
    # #     -G "${CMAKE_GENERATOR}"
    # #     -A "${CMAKE_GENERATOR_PLATFORM}"
    # #     -T "${CMAKE_GENERATOR_TOOLSET}"
    # #     ..
    # #     WORKING_DIRECTORY "${TARGET_DIR}/build")
    # #
    # # execute_process(COMMAND ${CMAKE_COMMAND}
    # #     --build .
    # #     --config ${CMAKE_BUILD_TYPE}
    # #     WORKING_DIRECTORY "${TARGET_DIR}/build")

    # Configure external project

    # CONFIGURE_COMMAND \"cmake -S . -B build\" # Disable all other steps
    # BUILD_COMMAND \"cmake --build build\"
    
    execute_process(
        COMMAND ${CMAKE_COMMAND} "${TARGET_DIR}"
        WORKING_DIRECTORY "${TARGET_DIR}"
    )

    # Build external project
    execute_process(
        COMMAND ${CMAKE_COMMAND} --build "${TARGET_DIR}"
    )

    # set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${TARGET_DIR}/build/${target}-prefix/src/${target}/cmake" PARENT_SCOPE)
    set(${target_DIR} "${TARGET_DIR}/${target}-prefix/src/${target}-build/" PARENT_SCOPE)
    # message(STATUS "${TARGET_DIR}/build/${target}-prefix/src/${target}-build/")
    # set_property(GLOBAL PROPERTY ${${hhhh}} "${TARGET_DIR}/build/${target}-prefix/src/${target}-build/")
endfunction()

# ===
# Mpi cmake modules
# ===

# build_external_project(Eigen3 Eigen3_DIR "https://gitlab.com/libeigen/eigen.git")
build_external_project(pybind11 pybind11_DIR "https://github.com/pybind/pybind11.git")
# build_external_project(mpi_cmake_modules mpi_cmake_modules_DIR "https://github.com/machines-in-motion/mpi_cmake_modules.git")

# set(${CMAKE_PREFIX_PATH} "${CMAKE_BINARY_DIR}/_deps/mpi_cmake_modules/build/mpi_cmake_modules-prefix/src/mpi_cmake_modules-build")
message(STATUS ${Eigen3_DIR})
message(STATUS ${pybind11_DIR})
message(STATUS ${mpi_cmake_modules_DIR})

# find_package(mpi_cmake_modules REQUIRED)
# find_package(pybind11 REQUIRED)
# find_package(Eigen3 REQUIRED)

# ===
# Real time tools
# ===

# FetchContent_Declare(
#   real_time_tools
#   # Specify the commit you depend on and update it regularly.
#   GIT_REPOSITORY https://github.com/machines-in-motion/real_time_tools.git
# )

# add_dependencies(real_time_tools mpi_cmake_modules pybind11 eigen3)

# FetchContent_MakeAvailable(real_time_tools QUIET)
# # FetchContent_MakeAvailable(mpi_cmake_modules real_time_tools)
