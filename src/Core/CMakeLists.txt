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


target_link_libraries(MonopodSdkCore real_time_tools::real_time_tools)
target_link_libraries(MonopodSdkCore time_series::time_series)
target_link_libraries(MonopodSdkCore Eigen3::Eigen)
target_link_libraries(MonopodSdkCore Threads::Threads)
target_link_libraries(MonopodSdkCore MonopodDrivers)
target_link_libraries(MonopodSdkCore blmc_drivers)


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
    NAMESPACE MonopodSdkCore::
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    INSTALL_DESTINATION
    ${MONOPODSDK_INSTALL_LIBDIR}/cmake/MonopodSdkCore
)
