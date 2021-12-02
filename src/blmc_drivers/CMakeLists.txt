# =========
# Blmc Drivers
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

if(Xenomai_FOUND)
  add_definitions(${Xenomai_DEFINITIONS})
endif()

set(BLMC_DRIVERS_PUBLIC_HDRS
    include/monopod_sdk/blmc_drivers/devices/analog_sensor.hpp
    include/monopod_sdk/blmc_drivers/devices/can_bus.hpp
    include/monopod_sdk/blmc_drivers/devices/device_interface.hpp
    include/monopod_sdk/blmc_drivers/devices/leg.hpp
    include/monopod_sdk/blmc_drivers/devices/motor_board.hpp
    include/monopod_sdk/blmc_drivers/devices/motor.hpp
    include/monopod_sdk/blmc_drivers/utils/polynome.hpp
    include/monopod_sdk/blmc_drivers/serial_reader.hpp
    include/monopod_sdk/blmc_drivers/blmc_joint_module.hpp
)

add_library(blmc_drivers
    ${BLMC_DRIVERS_PUBLIC_HDRS}
     src/analog_sensors.cpp
     src/blmc_joint_module.cpp
     src/can_bus.cpp
     src/motor_board.cpp
     src/motor.cpp
     src/serial_reader.cpp
     src/utils/polynome.cpp
  )

if(Xenomai_FOUND)
  target_include_directories(${PROJECT_NAME} PUBLIC ${Xenomai_INCLUDE_DIR})
endif()

# Link the catkin dependencies.
add_dependencies(blmc_drivers rt::rt)
target_link_libraries(blmc_drivers real_time_tools::real_time_tools)
target_link_libraries(blmc_drivers time_series::time_series)
target_link_libraries(blmc_drivers Eigen3::Eigen)
target_link_libraries(blmc_drivers Threads::Threads)

# If on xenomai we need to link to the real time os librairies.
if(Xenomai_FOUND)
  target_link_libraries(blmc_drivers ${Xenomai_LIBRARY_XENOMAI}
                        ${Xenomai_LIBRARY_NATIVE} ${Xenomai_LIBRARY_RTDM})
endif()

add_library(blmc_drivers::blmc_drivers ALIAS blmc_drivers)

target_include_directories(blmc_drivers PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${MONOPODSDK_INSTALL_INCLUDEDIR}>)

set_target_properties(blmc_drivers PROPERTIES
PUBLIC_HEADER "${BLMC_DRIVERS_PUBLIC_HDRS}")


#
# Manage exectuables
#

add_executable(can_encoder_index_test
    src/programs/can_encoder_index_test.cpp
)
target_link_libraries(can_encoder_index_test
    blmc_drivers
)

set(misc_targets)
list(APPEND misc_targets can_encoder_index_test)
macro(add_demo demo_name)
  add_executable(
    ${demo_name}
    demos/pd_control.cpp demos/sine_torque_control.cpp
    demos/sine_position_control.cpp demos/const_torque_control.cpp
    demos/${demo_name}.cpp)
  target_include_directories(
    ${demo_name}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/demos>
         $<INSTALL_INTERFACE:${MONOPODSDK_INSTALL_INCLUDEDIR}>)
  target_link_libraries(${demo_name} blmc_drivers)
  list(APPEND misc_targets ${demo_name})
endmacro()

add_demo(demo_sine_torque_1_motor)
add_demo(demo_sine_position_1_motor)
add_demo(demo_const_torque_1_motor)
add_demo(demo_single_board)
add_demo(demo_leg)
add_demo(demo_2_motors)
add_demo(demo_1_motor_print_everything)
add_demo(demo_1_motor)
add_demo(demo_8_motors)
add_demo(demo_print_analog_sensors)


install(
    TARGETS
    blmc_drivers ${misc_targets}
    EXPORT blmc_driversExport
    LIBRARY DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${MONOPODSDK_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${MONOPODSDK_INSTALL_INCLUDEDIR}/monopod_sdk/blmc_drivers)


install_basic_package_files(blmc_drivers
    COMPONENT blmc_drivers
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT blmc_driversExport
    NAMESPACE blmc_drivers::
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    INSTALL_DESTINATION
    ${MONOPODSDK_INSTALL_LIBDIR}/cmake/monopod_sdk/blmc_drivers
)
