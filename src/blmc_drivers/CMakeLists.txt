if(Xenomai_FOUND)
  add_definitions(${Xenomai_DEFINITIONS})
endif()

# ================
# Blmc Drivers / Devices
# ================

set(BLMC_DEVICES_PUBLIC_HDRS
    include/monopod_sdk/blmc_drivers/devices/analog_sensor.hpp
    include/monopod_sdk/blmc_drivers/devices/can_bus.hpp
    include/monopod_sdk/blmc_drivers/devices/device_interface.hpp
    include/monopod_sdk/blmc_drivers/devices/motor_board.hpp
    include/monopod_sdk/blmc_drivers/devices/motor.hpp
    )

add_library(devices
            ${BLMC_DRIVERS_PUBLIC_HDRS}
             src/analog_sensors.cpp
             src/can_bus.cpp
             src/motor_board.cpp
             src/motor.cpp)
add_library(blmc_drivers::devices ALIAS devices)

target_include_directories(devices INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${MONOPODSDK_INSTALL_INCLUDEDIR}>)

add_dependencies(devices rt::rt)
target_link_libraries( devices
    PUBLIC
    real_time_tools::real_time_tools
    time_series::time_series
    Eigen3::Eigen
    Threads::Threads)
# If on xenomai we need to link to the real time os librairies.
if(Xenomai_FOUND)
  target_link_libraries(devices ${Xenomai_LIBRARY_XENOMAI}
                        ${Xenomai_LIBRARY_NATIVE} ${Xenomai_LIBRARY_RTDM})
endif()

set_target_properties(devices PROPERTIES
    PUBLIC_HEADER "${BLMC_DEVICES_PUBLIC_HDRS}")


# ================
# Blmc Drivers / Utils
# ================

set(BLMC_UTILS_PUBLIC_HDRS
    include/monopod_sdk/blmc_drivers/utils/polynome.hpp
    )

add_library(utils
            ${BLMC_UTILS_PUBLIC_HDRS}
             src/utils/polynome.cpp)

add_library(blmc_drivers::utils ALIAS utils)

target_include_directories(utils INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${MONOPODSDK_INSTALL_INCLUDEDIR}>)

add_dependencies(utils rt::rt)
target_link_libraries( utils
    PUBLIC
    real_time_tools::real_time_tools
    time_series::time_series
    Eigen3::Eigen
    Threads::Threads)
# If on xenomai we need to link to the real time os librairies.

if(Xenomai_FOUND)
  target_link_libraries(utils ${Xenomai_LIBRARY_XENOMAI}
                        ${Xenomai_LIBRARY_NATIVE} ${Xenomai_LIBRARY_RTDM})
endif()

set_target_properties(utils PROPERTIES
    PUBLIC_HEADER "${BLMC_UTILS_PUBLIC_HDRS}")


# =========
# Blmc Drivers
# =========

set(BLMC_DRIVERS_PUBLIC_HDRS
    include/monopod_sdk/blmc_drivers/serial_reader.hpp
    include/monopod_sdk/blmc_drivers/blmc_joint_module.hpp
)

add_library(blmc_drivers
    ${BLMC_DRIVERS_PUBLIC_HDRS}
     src/blmc_joint_module.cpp
     src/serial_reader.cpp
  )

if(Xenomai_FOUND)
  target_include_directories(blmc_drivers PUBLIC ${Xenomai_INCLUDE_DIR})
endif()


add_library(blmc_drivers::blmc_drivers ALIAS blmc_drivers)

target_include_directories(blmc_drivers PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${MONOPODSDK_INSTALL_INCLUDEDIR}>)

set_target_properties(blmc_drivers PROPERTIES
    PUBLIC_HEADER "${BLMC_DRIVERS_PUBLIC_HDRS}")


# ===========================
# Manage Exectuables
# ===========================

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
add_demo(demo_2_motors)
add_demo(demo_1_motor_print_everything)
add_demo(demo_1_motor)
add_demo(demo_8_motors)
add_demo(demo_print_analog_sensors)
# add_demo(demo_leg)


install(
    TARGETS
    blmc_drivers ${misc_targets}
    EXPORT blmc_driversExport
    LIBRARY DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${MONOPODSDK_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${MONOPODSDK_INSTALL_INCLUDEDIR}/monopod_sdk/blmc_drivers)

install(
    TARGETS
    utils
    EXPORT blmc_driversExport
    LIBRARY DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${MONOPODSDK_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${MONOPODSDK_INSTALL_INCLUDEDIR}/monopod_sdk/blmc_drivers/utils)

install(
    TARGETS
    devices
    EXPORT blmc_driversExport
    LIBRARY DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${MONOPODSDK_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${MONOPODSDK_INSTALL_INCLUDEDIR}/monopod_sdk/blmc_drivers/devices)

install_basic_package_files(blmc_drivers
    COMPONENT blmc_drivers
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT blmc_driversExport
    NAMESPACE blmc_drivers::
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    INSTALL_DESTINATION
    ${MONOPODSDK_INSTALL_LIBDIR}/cmake/blmc_drivers
)
