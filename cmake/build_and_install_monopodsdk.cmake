
# ================
# MonopodSdk::Utils
# ================

    # set(MONOPOD_UTILS_PUBLIC_HDRS
    #     include/monopod_sdk/monopod_drivers/utils/os_interface.hpp)
    #
    # add_library(utils INTERFACE)
    #
    # add_library(MonopodSdk::utils ALIAS utils)
    #
    # target_include_directories(utils INTERFACE
    #     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    #     $<INSTALL_INTERFACE:${MONOPODSDK_INSTALL_INCLUDEDIR}>)
    #
    # add_dependencies(utils rt::rt)
    #
    # target_link_libraries(utils
    #     INTERFACE
    #     real_time_tools::real_time_tools
    #     Threads::Threads)
    # # If on xenomai we need to link to the real time os librairies.
    #
    # if(Xenomai_FOUND)
    #   target_link_libraries(utils
    #     ${Xenomai_LIBRARY_XENOMAI}
    #     ${Xenomai_LIBRARY_NATIVE}
    #     ${Xenomai_LIBRARY_RTDM})
    # endif()
    #
    # set_target_properties(utils PROPERTIES PUBLIC_HEADER "${MONOPOD_UTILS_PUBLIC_HDRS}")
    # add_custom_target(utils_headers SOURCES ${MONOPOD_UTILS_PUBLIC_HDRS})

    # ================
    # Blmc Drivers / Utils
    # ================

    set(MONOPOD_UTILS_PUBLIC_HDRS
        include/monopod_sdk/monopod_drivers/utils/polynome.hpp
        include/monopod_sdk/monopod_drivers/utils/polynome.hxx
        include/monopod_sdk/monopod_drivers/utils/os_interface.hpp
        )

    add_library(utils
                ${MONOPOD_UTILS_PUBLIC_HDRS}
                 src/utils/polynome.cpp)

    add_library(MonopodSdk::utils ALIAS utils)

    target_include_directories(utils PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${MONOPODSDK_INSTALL_INCLUDEDIR}>)

    add_dependencies(utils rt::rt)

    target_link_libraries(utils
        PUBLIC
        real_time_tools::real_time_tools)
    # If on xenomai we need to link to the real time os librairies.

    if(Xenomai_FOUND)
      target_link_libraries(utils ${Xenomai_LIBRARY_XENOMAI}
                            ${Xenomai_LIBRARY_NATIVE} ${Xenomai_LIBRARY_RTDM})
    endif()

    set_target_properties(utils PROPERTIES PUBLIC_HEADER "${MONOPOD_UTILS_PUBLIC_HDRS}")

# ================
# MonopodSdk::Devices
# ================

    set(MONOPOD_DEVICES_PUBLIC_HDRS
        include/monopod_sdk/monopod_drivers/devices/device_interface.hpp
        include/monopod_sdk/monopod_drivers/devices/boards.hpp
        # Changed stuff
        include/monopod_sdk/monopod_drivers/devices/can_bus.hpp
        include/monopod_sdk/monopod_drivers/devices/motor.hpp
        include/monopod_sdk/monopod_drivers/devices/encoder.hpp
        )

    add_library(devices
      ${MONOPOD_DEVICES_PUBLIC_HDRS}
      src/boards.cpp
      # Changed stuff
      src/can_bus.cpp
      src/motor.cpp
      src/encoder.cpp
    )

    add_library(MonopodSdk::devices ALIAS devices)

    target_include_directories(devices PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${MONOPODSDK_INSTALL_INCLUDEDIR}>)

    add_dependencies(devices rt::rt)
    target_link_libraries(devices PUBLIC
        utils
        real_time_tools::real_time_tools
        time_series::time_series
        Eigen3::Eigen
        Threads::Threads
      )
    # If on xenomai we need to link to the real time os librairies.
    if(Xenomai_FOUND)
      target_link_libraries(devices
        ${Xenomai_LIBRARY_XENOMAI}
        ${Xenomai_LIBRARY_NATIVE}
        ${Xenomai_LIBRARY_RTDM})
    endif()

    set_target_properties(devices PROPERTIES PUBLIC_HEADER "${MONOPOD_DEVICES_PUBLIC_HDRS}")

# ==============================================================================
# MonopodSdk::MonopodDrivers
# ==============================================================================

  # ===============================
  # add library
  # ===============================
  set(MONOPOD_DRIVERS_PUBLIC_HDRS
      include/monopod_sdk/monopod_drivers/leg.hpp
      include/monopod_sdk/monopod_drivers/planarizer.hpp
      include/monopod_sdk/monopod_drivers/motor_joint_module.hpp
      include/monopod_sdk/monopod_drivers/encoder_joint_module.hpp
  )
  add_library(MonopodDrivers
      ${MONOPOD_DRIVERS_PUBLIC_HDRS}
      src/motor_joint_module.cpp
      src/encoder_joint_module.cpp
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
      devices
      utils
      PRIVATE
      real_time_tools::real_time_tools
      time_series::time_series)

  # ===============================
  # Build Library
  # ===============================
  add_library(MonopodSdk::MonopodDrivers ALIAS MonopodDrivers)

  target_include_directories(MonopodDrivers PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${MONOPODSDK_INSTALL_INCLUDEDIR}>
  )

  set_target_properties(MonopodDrivers PROPERTIES
    PUBLIC_HEADER "${MONOPOD_DRIVERS_PUBLIC_HDRS}"
  )

# ==============================================================================
# MonopodSdk::Monopodsdk
# ==============================================================================

  set(MONOPOD_SDK_CORE_PUBLIC_HDRS
    include/monopod_sdk/monopod.hpp
    include/monopod_sdk/common_header.hpp
  )

  add_library(MonopodSdk
    ${MONOPOD_SDK_CORE_PUBLIC_HDRS}
    src/monopod.cpp
  )

  add_library(MonopodSdk::MonopodSdk ALIAS MonopodSdk)

  target_include_directories(MonopodSdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${MONOPODSDK_INSTALL_INCLUDEDIR}>
  )

  set_target_properties(MonopodSdk PROPERTIES
    PUBLIC_HEADER "${MONOPOD_SDK_CORE_PUBLIC_HDRS}"
  )


  # ===============================
  # Link Libraries
  # ===============================

  target_link_libraries(MonopodSdk
    PUBLIC
    devices
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
      target_link_libraries(${demo_name}
        MonopodDrivers
        MonopodSdk
        utils
        devices
      )
      list(APPEND misc_targets ${demo_name})
    endmacro()

    # add_demo(demo_leg_sine_position)
    add_demo(demo_print_position_sdk)

# ===============================
# Install
# ===============================

    install(
      TARGETS
      MonopodSdk ${misc_targets}
      EXPORT MonopodSdkExport
      LIBRARY DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
      ARCHIVE DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
      RUNTIME DESTINATION ${MONOPODSDK_INSTALL_BINDIR}
      PUBLIC_HEADER DESTINATION ${MONOPODSDK_INSTALL_INCLUDEDIR}/monopod_sdk
    )

    install(
        TARGETS
        MonopodDrivers
        EXPORT MonopodSdkExport
        LIBRARY DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${MONOPODSDK_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${MONOPODSDK_INSTALL_INCLUDEDIR}/monopod_sdk/monopod_drivers
      )

    install(
        TARGETS
        utils
        EXPORT MonopodSdkExport
        LIBRARY DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${MONOPODSDK_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${MONOPODSDK_INSTALL_INCLUDEDIR}/monopod_sdk/monopod_drivers/utils
      )

    install(
        TARGETS
        devices
        EXPORT MonopodSdkExport
        LIBRARY DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${MONOPODSDK_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${MONOPODSDK_INSTALL_INCLUDEDIR}/monopod_sdk/monopod_drivers/devices
      )

    install_basic_package_files(MonopodSdk
        COMPONENT MonopodSdk
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
        EXPORT MonopodSdkExport
        DEPENDENCIES real_time_tools time_series
        NAMESPACE MonopodSdk::
        NO_CHECK_REQUIRED_COMPONENTS_MACRO
        INSTALL_DESTINATION
        ${MONOPODSDK_INSTALL_LIBDIR}/cmake/MonopodSdk
      )
