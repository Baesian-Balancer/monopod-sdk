# ==============
# Find Monopod SDK dependencies
# ==============
# set(MONOPODSDK_COMPONENTS BlmcDrivers MonopodDrivers)
set(MONOPODSDK_COMPONENTS MonopodDrivers)
set(MONOPODSDK_PRIVATE_DEPENDENCIES time_series real_time_tools)

# add_subdirectory(blmc_drivers)
add_subdirectory(monopod_drivers)

# Dummy meta target for the top-level EXPORT
add_library(MonopodSdk INTERFACE)

# =================================================================
# Install is not something supported now. need to fix build
# dependencies first.
# =================================================================

install(TARGETS MonopodSdk EXPORT MonopodSdkExport)

install_basic_package_files(MonopodSdk
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT MonopodSdkExport
    DEPENDENCIES ${MONOPODSDK_COMPONENTS}
    PRIVATE_DEPENDENCIES ${MONOPODSDK_PRIVATE_DEPENDENCIES}
    NAMESPACE MonopodSdk::
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    INSTALL_DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}/cmake/MonopodSdk)
