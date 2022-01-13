# ==============================================================================
# Monopod Drivers
# ==============================================================================

  # ===============================
  # add library
  # ===============================
  set(MONOPOD_DRIVERS_PUBLIC_HDRS
      include/monopod_sdk/monopod_drivers/leg.hpp
      include/monopod_sdk/monopod_drivers/planarizer.hpp
  )
  add_library(MonopodDrivers
      ${MONOPOD_DRIVERS_PUBLIC_HDRS}
          # src/encoder.cpp
          # src/planarizer.cpp
    )

  set_target_properties(MonopodDrivers PROPERTIES LINKER_LANGUAGE CXX)

  # ===============================
  # Set depends and link libraries
  # ===============================
  if(Xenomai_FOUND)
    add_definitions(${Xenomai_DEFINITIONS})
    target_include_directories(MonopodDrivers PUBLIC ${Xenomai_INCLUDE_DIR})
    # If on xenomai we need to link to the real time os librairies.
    target_link_libraries(MonopodDrivers
      ${Xenomai_LIBRARY_XENOMAI}
      ${Xenomai_LIBRARY_NATIVE}
      ${Xenomai_LIBRARY_RTDM}
    )
  endif()

  add_dependencies(MonopodDrivers rt::rt)

  target_link_libraries(MonopodDrivers
      PUBLIC
      BlmcDrivers::BlmcDrivers
      BlmcDrivers::devices
      BlmcDrivers::utils
      PRIVATE
      real_time_tools::real_time_tools
      time_series::time_series)

  # ===============================
  # Build Library
  # ===============================
  add_library(MonopodDrivers::MonopodDrivers ALIAS MonopodDrivers)

  target_include_directories(MonopodDrivers PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${MONOPODSDK_INSTALL_INCLUDEDIR}>
  )

  set_target_properties(MonopodDrivers PROPERTIES
    PUBLIC_HEADER "${MONOPOD_DRIVERS_PUBLIC_HDRS}"
  )

# ==============================================================================
# Monopod sdk core
# ==============================================================================

  set(MONOPOD_SDK_CORE_PUBLIC_HDRS
    include/monopod_sdk/monopod.hpp
  )

  add_library(MonopodSdkCore
    ${MONOPOD_SDK_CORE_PUBLIC_HDRS}
    src/monopod.cpp
  )

  add_library(MonopodDrivers::MonopodSdkCore ALIAS MonopodSdkCore)

  target_include_directories(MonopodSdkCore PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${MONOPODSDK_INSTALL_INCLUDEDIR}>
  )

  set_target_properties(MonopodSdkCore PROPERTIES
    PUBLIC_HEADER "${MONOPOD_SDK_CORE_PUBLIC_HDRS}"
  )


  # ===============================
  # Link Libraries
  # ===============================

  target_link_libraries(MonopodSdkCore
    PUBLIC
    BlmcDrivers::BlmcDrivers
    BlmcDrivers::devices
    BlmcDrivers::utils
    MonopodDrivers
    PRIVATE
    time_series::time_series
    real_time_tools::real_time_tools
    Eigen3::Eigen
    Threads::Threads
    )


# ===============================
# Demos
# ===============================

set(misc_targets)
list(APPEND misc_targets)
macro(add_demo demo_name)
  add_executable(
    ${demo_name}
    demos/sine_position_control.cpp
    demos/${demo_name}.cpp)
  target_include_directories(${demo_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/demos>
    $<INSTALL_INTERFACE:${MONOPODSDK_INSTALL_INCLUDEDIR}>
  )
  target_link_libraries(${demo_name} MonopodDrivers MonopodSdkCore)
  list(APPEND misc_targets ${demo_name})
endmacro()

add_demo(demo_leg_sine_position)
add_demo(demo_planarizer)
add_demo(demo_threads)
# add_demo(demo_inference)

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
    PUBLIC_HEADER DESTINATION ${MONOPODSDK_INSTALL_INCLUDEDIR}/monopod_sdk/monopod_drivers
  )

install(
    TARGETS
    MonopodSdkCore
    EXPORT MonopodDriversExport
    LIBRARY DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${MONOPODSDK_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${MONOPODSDK_INSTALL_INCLUDEDIR}/monopod_sdk
  )

install_basic_package_files(MonopodDrivers
    COMPONENT MonopodDrivers
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT MonopodDriversExport
    DEPENDENCIES BlmcDrivers real_time_tools time_series
    NAMESPACE MonopodDrivers::
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    INSTALL_DESTINATION
    ${MONOPODSDK_INSTALL_LIBDIR}/cmake/MonopodDrivers
  )
