# =========
# Monopod Drivers
# =========

# Usual dependencies.
find_package(mpi_cmake_modules REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(time_series REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rt REQUIRED)
find_package(Threads REQUIRED)
# Check for xenomai as these drivers are xenomai compatible.
find_package(Xenomai QUIET)

# ===============================
# add library
# ===============================

set(MONOPOD_DRIVERS_PUBLIC_HDRS

)

add_library(monopod_drivers
    ${MONOPOD_DRIVERS_PUBLIC_HDRS}

  )

# ===============================
# Set depends and link libraries
# ===============================

if(Xenomai_FOUND)
  add_definitions(${Xenomai_DEFINITIONS})
  target_include_directories(monopod_drivers PUBLIC ${Xenomai_INCLUDE_DIR})
  # If on xenomai we need to link to the real time os librairies.
  target_link_libraries(monopod_drivers ${Xenomai_LIBRARY_XENOMAI}
                        ${Xenomai_LIBRARY_NATIVE} ${Xenomai_LIBRARY_RTDM})
endif()

# Link the catkin dependencies.
add_dependencies(monopod_drivers rt::rt)
target_link_libraries(monopod_drivers real_time_tools::real_time_tools)
target_link_libraries(monopod_drivers time_series::time_series)
target_link_libraries(monopod_drivers Eigen3::Eigen)
target_link_libraries(monopod_drivers Threads::Threads)


# ===============================
# Build Library
# ===============================

add_library(monopod_drivers::monopod_drivers ALIAS monopod_drivers)

target_include_directories(monopod_drivers PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${MONOPODSDK_INSTALL_INCLUDEDIR}>)

set_target_properties(monopod_drivers PROPERTIES
PUBLIC_HEADER "${MONOPOD_DRIVERS_PUBLIC_HDRS}")

# ===============================
# Demos
# ===============================

set(misc_targets)
list(APPEND misc_targets can_encoder_index_test)
macro(add_demo demo_name)
  add_executable(
    ${demo_name}
    # demos/pd_control.cpp demos/sine_torque_control.cpp
    # demos/sine_position_control.cpp demos/const_torque_control.cpp
    demos/${demo_name}.cpp)
  target_include_directories(
    ${demo_name}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/demos>
         $<INSTALL_INTERFACE:${MONOPODSDK_INSTALL_INCLUDEDIR}>)
  target_link_libraries(${demo_name} monopod_drivers)
  list(APPEND misc_targets ${demo_name})
endmacro()

# add_demo(demo_sine_torque_1_motor)

# ===============================
# Install
# ===============================

install(
    TARGETS
    monopod_drivers ${misc_targets}
    EXPORT monopod_driversExport
    LIBRARY DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${MONOPODSDK_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${MONOPODSDK_INSTALL_INCLUDEDIR}/monopod_sdk/monopod_drivers)


install_basic_package_files(monopod_drivers
    COMPONENT monopod_drivers
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT monopod_driversExport
    NAMESPACE monopod_drivers::
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    INSTALL_DESTINATION
    ${MONOPODSDK_INSTALL_LIBDIR}/cmake/monopod_sdk/monopod_drivers
)
