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


install(
    TARGETS
    blmc_drivers can_encoder_index_test
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
