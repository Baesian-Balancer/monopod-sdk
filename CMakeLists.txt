cmake_minimum_required(VERSION 3.16)
project(monopod_sdk VERSION 0.0.1)

# Add custom functions / macros
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Usual dependencies.
find_package(mpi_cmake_modules REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(time_series REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rt REQUIRED)
find_package(Threads REQUIRED)
# Check for xenomai as these drivers are xenomai compatible.
find_package(Xenomai QUIET)

# Include useful features
include(GNUInstallDirs)

# Set CMAKE_BUILD_TYPE to set to debug
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING
            "Choose the type of build, options are: Debug or Release" FORCE)
    endif()
    set(MONOPODSDK_BUILD_TYPES "Debug" "Release")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${MONOPODSDK_BUILD_TYPES})
endif()

# Expose shared or static compilation
set(MONOPODSDK_BUILD_SHARED_LIBRARY TRUE
    CACHE BOOL "Compile libraries as shared libraries")
set(BUILD_SHARED_LIBS ${MONOPODSDK_BUILD_SHARED_LIBRARY})

# Control where binaries and libraries are placed in the build folder.
# This simplifies tests running in Windows.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

# =========
# Use -fPIC even if statically compiled
# Makes the machine code use relative addressing not absolute
# Needed for libraries.
# https://stackoverflow.com/questions/5311515/gcc-fpic-option
# =========

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Tweak linker flags in Linux
if(UNIX AND NOT APPLE)
    if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
        get_filename_component(LINKER_BIN ${CMAKE_LINKER} NAME)
            if("${LINKER_BIN}" STREQUAL "ld")
                set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--unresolved-symbols=report-all")
            endif()
    endif()
endif()

# =========
# Get include-what-you-use information when compiling
# https://include-what-you-use.org/
# =========

option(MONOPODSDK_USE_IWYU "Get the output of include-what-you-use" OFF)
mark_as_advanced(MONOPODSDK_USE_IWYU)
if(MONOPODSDK_USE_IWYU)
    find_program(IWYU_PATH NAMES include-what-you-use iwyu)
    if(IWYU_PATH)
        set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH})
    endif()
endif()

if(${CMAKE_VERSION} VERSION_GREATER 3.15)
    cmake_policy(SET CMP0094 NEW)
endif()

# =========
# Dependencies
# =========

add_subdirectory(deps)


# Select the appropriate install prefix used throughout the project
set(MONOPODSDK_INSTALL_BINDIR ${CMAKE_INSTALL_BINDIR})
set(MONOPODSDK_INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR})
set(MONOPODSDK_INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR})

# # Add the libraries installed in the Python site-package folder
# # (that in this case is CMAKE_INSTALL_PREFIX)
# set(EXTRA_RPATH_DIRS
#     "${CMAKE_INSTALL_PREFIX}"
#     "${CMAKE_INSTALL_PREFIX}/scenario/bindings")
#
#
# # Configure RPATH
# include(AddInstallRPATHSupport)
# add_install_rpath_support(
#     BIN_DIRS
#     "${CMAKE_INSTALL_PREFIX}/${SCENARIO_INSTALL_BINDIR}"
#     LIB_DIRS
#     "${CMAKE_INSTALL_PREFIX}/${SCENARIO_INSTALL_LIBDIR}"
#     "${CMAKE_INSTALL_PREFIX}/${SCENARIO_INSTALL_LIBDIR}/scenario/plugins"
#     "${EXTRA_RPATH_DIRS}"
#     INSTALL_NAME_DIR
#     "${CMAKE_INSTALL_PREFIX}/${SCENARIO_INSTALL_LIBDIR}"
#     DEPENDS ENABLE_RPATH
#     USE_LINK_PATH)

# Helper for exporting targets
# talking about this package here
# http://robotology.github.io/ycm/gh-pages/git-master/module/InstallBasicPackageFiles.html

include(InstallBasicPackageFiles)

# =========
# Monopod SDK
# =========
add_subdirectory(src)

# # ========
# # DOCS
# # ========

if(BUILD_DOCS)
    add_subdirectory(docs)
endif()

# ===================
# Add unistall target
# ===================
include(AddUninstallTarget)
