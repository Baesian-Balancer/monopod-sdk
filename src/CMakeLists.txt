# ==============
# Find Monopod SDK dependencies
# ==============
set(MONOPODSDK_COMPONENTS MonopodSDK BlmcDrivers MonopodDrivers)

set(MONOPODSDK_PRIVATE_DEPENDENCIES pybind11 Eigen3 mpi_cmake_modules
    real_time_tools signal_handler shared_memory time_series)

# add_subdirectory(blmc_drivers)
# add_subdirectory(monopod_drivers)
# add_subdirectory(monopod_sdk)

# Dummy meta target for the top-level EXPORT
add_library(MonopodSDK INTERFACE)

# =================================================================
# Install is not something supported now. need to fix build
# dependencies first.
# =================================================================

# install(TARGETS MonopodSDK EXPORT MonopodSDKExport)
#
# install_basic_package_files(MonopodSDK
#     VERSION ${PROJECT_VERSION}
#     COMPATIBILITY AnyNewerVersion
#     EXPORT MonopodSDKExport
#     DEPENDENCIES ${MONOPODSDK_COMPONENTS}
#     PRIVATE_DEPENDENCIES ${MONOPODSDK_PRIVATE_DEPENDENCIES}
#     NAMESPACE MonopodSDK::
#     NO_CHECK_REQUIRED_COMPONENTS_MACRO
#     INSTALL_DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}/cmake/MonopodSDK)
