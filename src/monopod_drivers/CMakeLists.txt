# =========
# Monopod Drivers
# =========

# ===============================
# add library
# ===============================

set(MONOPOD_DRIVERS_PUBLIC_HDRS
    include/monopod_sdk/monopod_drivers/encoder.hpp
    include/monopod_sdk/monopod_drivers/leg.hpp
    include/monopod_sdk/monopod_drivers/planarizer.hpp
)

add_library(MonopodDrivers
    ${MONOPOD_DRIVERS_PUBLIC_HDRS}
        src/encoder.cpp
        src/planarizer.cpp
  )

# ===============================
# Set depends and link libraries
# ===============================

if(Xenomai_FOUND)
  add_definitions(${Xenomai_DEFINITIONS})
  target_include_directories(MonopodDrivers PUBLIC ${Xenomai_INCLUDE_DIR})
  # If on xenomai we need to link to the real time os librairies.
  target_link_libraries(MonopodDrivers ${Xenomai_LIBRARY_XENOMAI}
                        ${Xenomai_LIBRARY_NATIVE} ${Xenomai_LIBRARY_RTDM})
endif()

# Link the catkin dependencies.
add_dependencies(MonopodDrivers rt::rt)
target_link_libraries(MonopodDrivers real_time_tools::real_time_tools)
target_link_libraries(MonopodDrivers time_series::time_series)
target_link_libraries(MonopodDrivers blmc_drivers::blmc_drivers)

# ===============================
# Build Library
# ===============================

add_library(MonopodDrivers::MonopodDrivers ALIAS MonopodDrivers)

target_include_directories(MonopodDrivers PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${MONOPODSDK_INSTALL_INCLUDEDIR}>)

set_target_properties(MonopodDrivers PROPERTIES
PUBLIC_HEADER "${MONOPOD_DRIVERS_PUBLIC_HDRS}")

# ===============================
# Demos
# ===============================

set(misc_targets)
list(APPEND misc_targets can_encoder_index_test)
macro(add_demo demo_name)
  add_executable(
    ${demo_name}
    demos/sine_position_control.cpp
    demos/${demo_name}.cpp)
  target_include_directories(
    ${demo_name}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/demos>
         $<INSTALL_INTERFACE:${MONOPODSDK_INSTALL_INCLUDEDIR}>)
  target_link_libraries(${demo_name} MonopodDrivers)
  list(APPEND misc_targets ${demo_name})
endmacro()

add_demo(demo_leg_sine_position)
add_demo(demo_leg)
add_demo(demo_planarizer)

# ===============================
# Install
# ===============================

install(
    TARGETS
    MonopodDrivers ${misc_targets}
    EXPORT MonopodDriversExport
    LIBRARY DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${MONOPODSDK_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${MONOPODSDK_INSTALL_INCLUDEDIR}/monopod_sdk/monopod_drivers)


install_basic_package_files(MonopodDrivers
    COMPONENT MonopodDrivers
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT MonopodDriversExport
    NAMESPACE MonopodDrivers::
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    INSTALL_DESTINATION
    ${MONOPODSDK_INSTALL_LIBDIR}/cmake/MonopodDrivers
)
