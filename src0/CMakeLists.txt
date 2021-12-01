set(PUBLIC_HDRS
include/myproject/test.h)

add_library(TestDeps
    ${PUBLIC_HDRS}
    src/test.cpp)

add_library(TestDeps::TestDeps ALIAS TestDeps)

target_include_directories(TestDeps PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${TEST_INSTALL_INCLUDEDIR}>)

find_package(real_time_tools REQUIRED)

target_link_libraries(TestDeps PUBLIC real_time_tools::real_time_tools)

set_target_properties(TestDeps PROPERTIES
    PUBLIC_HEADER "${PUBLIC_HDRS}")

install(
    TARGETS
    TestDeps
    EXPORT TestDepsExport
    LIBRARY DESTINATION ${TEST_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${TEST_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${TEST_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${TEST_INSTALL_INCLUDEDIR}/myproject)


install_basic_package_files(TestDeps
    # COMPONENT TestCode
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT TestDepsExport
    DEPENDENCIES real_time_tools
    NAMESPACE TestDeps::
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
    INSTALL_DESTINATION ${TEST_INSTALL_LIBDIR}/cmake/TestDeps
)
