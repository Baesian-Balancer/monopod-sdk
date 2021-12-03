# ==============
# Find Monopod SDK dependencies
# ==============
set(MONOPODSDK_COMPONENTS Core blmc_drivers monopod_drivers)

# set(MONOPODSDK_PRIVATE_DEPENDENCIES pybind11 Eigen3 mpi_cmake_modules
#     real_time_tools signal_handler shared_memory time_series)

add_subdirectory(blmc_drivers)
add_subdirectory(monopod_drivers)
# add_subdirectory(Core)

# Dummy meta target for the top-level EXPORT
add_library(monopod_sdk INTERFACE)

# =================================================================
# Install is not something supported now. need to fix build
# dependencies first.
# =================================================================

install(TARGETS monopod_sdk EXPORT monopod_sdkExport)

install_basic_package_files(monopod_sdk
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT monopod_sdkExport
    DEPENDENCIES ${MONOPODSDK_COMPONENTS}
    PRIVATE_DEPENDENCIES ${MONOPODSDK_PRIVATE_DEPENDENCIES}
    NAMESPACE monopod_sdk::
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    INSTALL_DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}/cmake/monopod_sdk)
