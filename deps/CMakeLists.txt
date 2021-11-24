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

# message(STATUS ${FETCHCONTENT_BASE_DIR})

# =============================================================================
# EXTERNAL PROJECT DEPENDS - I hate my life :)
# =============================================================================

function (build_external_project target target_ROOT git_repo)
    # Set up CMakeLists.txt in target directory
    set(TARGET_DIR "${${target_ROOT}}")
    list(APPEND ARGN -DCMAKE_INSTALL_PREFIX=${TARGET_DIR})
    set(CMAKELIST_CONTENT
"cmake_minimum_required(VERSION ${CMAKE_MINIMUM_REQUIRED_VERSION})
project(build_external_project)
include(ExternalProject)
ExternalProject_add(${target}
    GIT_REPOSITORY \"${git_repo}\"
    BUILD_COMMAND cmake --build . --config ${CMAKE_BUILD_TYPE}
    CMAKE_ARGS ${ARGN})
add_custom_target(build_external_project)
add_dependencies(build_external_project ${target})")
    file(WRITE "${TARGET_DIR}/CMakeLists.txt" "${CMAKELIST_CONTENT}")
    #config external project
    execute_process(
        COMMAND ${CMAKE_COMMAND} .
        WORKING_DIRECTORY "${TARGET_DIR}"
    )
    # Build external project
    execute_process(
        COMMAND ${CMAKE_COMMAND} --build .
        WORKING_DIRECTORY "${TARGET_DIR}"
    )
endfunction()

set(pybind11_ROOT "${FETCHCONTENT_BASE_DIR}/pybind11" CACHE INTERNAL "")
build_external_project(pybind11 pybind11_ROOT "https://github.com/pybind/pybind11.git" -DPYBIND11_TEST=OFF -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
find_package(pybind11 REQUIRED)
list(APPEND GLOBAL_THIRDPARTY_LIB_ARGS "-Dpybind11_DIR=${pybind11_DIR}")

set(Eigen3_ROOT "${FETCHCONTENT_BASE_DIR}/Eigen3" CACHE INTERNAL "")
build_external_project(Eigen3 Eigen3_ROOT "https://gitlab.com/libeigen/eigen.git" -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
find_package(Eigen3 REQUIRED)
list(APPEND GLOBAL_THIRDPARTY_LIB_ARGS "-DEigen3_DIR=${Eigen3_DIR}")

set(mpi_cmake_modules_ROOT "${FETCHCONTENT_BASE_DIR}/mpi_cmake_modules" CACHE INTERNAL "")
build_external_project(mpi_cmake_modules mpi_cmake_modules_ROOT "https://github.com/machines-in-motion/mpi_cmake_modules.git" -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
find_package(mpi_cmake_modules REQUIRED)
list(APPEND GLOBAL_THIRDPARTY_LIB_ARGS "-Dmpi_cmake_modules_DIR=${mpi_cmake_modules_DIR}")

set(real_time_tools_ROOT "${FETCHCONTENT_BASE_DIR}/real_time_tools" CACHE INTERNAL "")
build_external_project(real_time_tools real_time_tools_ROOT "https://github.com/machines-in-motion/real_time_tools.git" -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} ${GLOBAL_THIRDPARTY_LIB_ARGS})
find_package(real_time_tools REQUIRED)
list(APPEND GLOBAL_THIRDPARTY_LIB_ARGS "-Dreal_time_tools_DIR=${real_time_tools_DIR}")

set(signal_handler_ROOT "${FETCHCONTENT_BASE_DIR}/signal_handler" CACHE INTERNAL "")
build_external_project(signal_handler signal_handler_ROOT "https://github.com/MPI-IS/signal_handler.git" -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} ${GLOBAL_THIRDPARTY_LIB_ARGS})
find_package(signal_handler REQUIRED)
list(APPEND GLOBAL_THIRDPARTY_LIB_ARGS "-Dsignal_handler_DIR=${signal_handler_DIR}")

set(shared_memory_ROOT "${FETCHCONTENT_BASE_DIR}/shared_memory" CACHE INTERNAL "")
build_external_project(shared_memory shared_memory_ROOT "https://github.com/machines-in-motion/shared_memory.git" -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} ${GLOBAL_THIRDPARTY_LIB_ARGS})
find_package(shared_memory REQUIRED)
list(APPEND GLOBAL_THIRDPARTY_LIB_ARGS "-Dshared_memory_DIR=${shared_memory_DIR}")

set(time_series_ROOT "${FETCHCONTENT_BASE_DIR}/time_series" CACHE INTERNAL "")
build_external_project(time_series time_series_ROOT "https://github.com/machines-in-motion/time_series.git" -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} ${GLOBAL_THIRDPARTY_LIB_ARGS})
find_package(time_series REQUIRED)
