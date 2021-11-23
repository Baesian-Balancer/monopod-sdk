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

function (build_external_project target git_repo)

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

    set(CMAKELIST_CONTENT "
        cmake_minimum_required(VERSION ${CMAKE_MINIMUM_REQUIRED_VERSION})
        project(build_external_project)
        include(ExternalProject)
        ExternalProject_add(${target}
            GIT_REPOSITORY \"${git_repo}\"
            #--Configure step-------------
            CONFIGURE_COMMAND \"\"
            #--Build step-----------------
            BUILD_COMMAND \"\"
            #--Install step---------------
            UPDATE_COMMAND \"\" # Skip annoying updates for every build
            INSTALL_COMMAND \"\"
            CMAKE_ARGS ${ARGN})
        add_custom_target(build_external_project)
        add_dependencies(build_external_project ${target})
    ")

    set(TARGET_DIR "${CMAKE_BINARY_DIR}/_deps/${target}")

    file(WRITE "${TARGET_DIR}/CMakeLists.txt" "${CMAKELIST_CONTENT}")

    file(MAKE_DIRECTORY "${TARGET_DIR}" "${TARGET_DIR}/build")

    # execute_process(COMMAND ${CMAKE_COMMAND}
    #     -G "${CMAKE_GENERATOR}"
    #     -A "${CMAKE_GENERATOR_PLATFORM}"
    #     -T "${CMAKE_GENERATOR_TOOLSET}"
    #     ..
    #     WORKING_DIRECTORY "${TARGET_DIR}/build")
    #
    # execute_process(COMMAND ${CMAKE_COMMAND}
    #     --build .
    #     --config ${CMAKE_BUILD_TYPE}
    #     WORKING_DIRECTORY "${TARGET_DIR}/build")

    # Configure external project
    execute_process(
        COMMAND ${CMAKE_COMMAND} ${TARGET_DIR}
        WORKING_DIRECTORY "${TARGET_DIR}/build"
    )

    # Build external project
    execute_process(
        COMMAND ${CMAKE_COMMAND} --build "${TARGET_DIR}/build"
    )

    set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${TARGET_DIR}/build/${target}-prefix/src/${target}/cmake")
    # set("${target}_DIR" "${TARGET_DIR}/build/${target}-prefix/src/${target}-build/" GLOBAL)
endfunction()

# ===
# Mpi cmake modules
# ===

build_external_project(mpi_cmake_modules "https://github.com/machines-in-motion/mpi_cmake_modules.git")
build_external_project(pybind11 "https://github.com/pybind/pybind11.git")


# # ===
# # Mpi cmake modules
# # ===
#
# FetchContent_Declare(
#   mpi_cmake_modules
#   # Specify the commit you depend on and update it regularly.
#   GIT_REPOSITORY https://github.com/machines-in-motion/mpi_cmake_modules.git
# )
# FetchContent_GetProperties(mpi_cmake_modules)
#
# if(NOT mpi_cmake_modules_POPULATED)
#     FetchContent_Populate(mpi_cmake_modules)
#
#     add_subdirectory(${mpi_cmake_modules_SOURCE_DIR}
#                      ${mpi_cmake_modules_BINARY_DIR}
#                      EXCLUDE_FROM_ALL)
# endif()
#
# # set(mpi_cmake_modules_DIR "${mpi_cmake_modules_SOURCE_DIR}/../mpi_cmake_modules-build";)
# # set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${mpi_cmake_modules_SOURCE_DIR}/cmake" PARENT_SCOPE)
#
# # ===
# # pybind11
# # ===
#
# FetchContent_Declare(
#   pybind11
#   # Specify the commit you depend on and update it regularly.
#   GIT_REPOSITORY https://github.com/pybind/pybind11.git
# )
# FetchContent_GetProperties(pybind11)
#
# if(NOT pybind11_POPULATED)
#     FetchContent_Populate(pybind11)
#
#     add_subdirectory(${pybind11_SOURCE_DIR}
#                      ${pybind11_BINARY_DIR}
#                      EXCLUDE_FROM_ALL)
# endif()
#
#
# # ===
# # Real time tools
# # ===
#
# FetchContent_Declare(
#   real_time_tools
#   # Specify the commit you depend on and update it regularly.
#   GIT_REPOSITORY https://github.com/machines-in-motion/real_time_tools.git
# )
#
# add_dependencies(real_time_tools mpi_cmake_modules pybind11)
#
# FetchContent_MakeAvailable(real_time_tools QUIET)
# # FetchContent_MakeAvailable(mpi_cmake_modules real_time_tools)
