# =========
# Monopod Drivers
# =========

set(MONOPOD_SDK_CORE_PUBLIC_HDRS
    include/monopod_sdk/monopod.hpp
)

add_library(MonopodSdkCore
    ${MONOPOD_SDK_CORE_PUBLIC_HDRS}
    src/monopod.cpp
  )

add_library(MonopodSdkCore::MonopodSdkCore ALIAS MonopodSdkCore)

target_include_directories(MonopodSdkCore PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${MONOPODSDK_INSTALL_INCLUDEDIR}>)

set_target_properties(MonopodSdkCore PROPERTIES
    PUBLIC_HEADER "${MONOPOD_SDK_CORE_PUBLIC_HDRS}")


target_link_libraries( MonopodSdkCore
    PUBLIC
    blmc_drivers::blmc_drivers
    blmc_drivers::devices
    blmc_drivers::utils
    MonopodDrivers::MonopodDrivers
    PRIVATE
    time_series::time_series
    real_time_tools::real_time_tools
    Eigen3::Eigen
    Threads::Threads)

install(
    TARGETS
    MonopodSdkCore
    EXPORT MonopodSdkCoreExport
    LIBRARY DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${MONOPODSDK_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${MONOPODSDK_INSTALL_INCLUDEDIR}/monopod_sdk)


install_basic_package_files(MonopodSdkCore
    COMPONENT Core
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT MonopodSdkCoreExport
    DEPENDENCIES blmc_drivers MonopodDrivers real_time_tools time_series
    NAMESPACE MonopodSdkCore::
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    INSTALL_DESTINATION
    ${MONOPODSDK_INSTALL_LIBDIR}/cmake/MonopodSdkCore
)
