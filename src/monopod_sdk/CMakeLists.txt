# # =========
# # Monopod SDK
# # =========
# #
# set(MONOPODSDK_PUBLIC_HDRS
#     include/monopodSDK/easylogging++.h)
#
# add_library(MonopodSDK
#     ${MONOPODSDK_PUBLIC_HDRS}
#     src/easylogging++.cc)
#
# add_library(MonopodSDK::MonopodSDK ALIAS MonopodSDK)
#
# target_include_directories(MonopodSDK PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:${MONOPODSDK_INSTALL_INCLUDEDIR}>)
#
# set_target_properties(MonopodSDK PROPERTIES
# PUBLIC_HEADER "${MONOPODSDK_PUBLIC_HDRS}")
#
#
# install(
#     TARGETS
#     MonopodSDK
#     EXPORT MonopodSDKExport
#     LIBRARY DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
#     ARCHIVE DESTINATION ${MONOPODSDK_INSTALL_LIBDIR}
#     RUNTIME DESTINATION ${MONOPODSDK_INSTALL_BINDIR}
#     PUBLIC_HEADER DESTINATION ${MONOPODSDK_INSTALL_INCLUDEDIR}/monopodSDK)
#
# # ===================
# # Install the targets
# # ===================
#
# install_basic_package_files(MonopodSDK
#     COMPONENT SDK
#     VERSION ${PROJECT_VERSION}
#     COMPATIBILITY AnyNewerVersion
#     EXPORT MonopodSDKExport
#     NAMESPACE MonopodSDK::
#     NO_CHECK_REQUIRED_COMPONENTS_MACRO
#     INSTALL_DESTINATION
#     ${MONOPODSDK_INSTALL_LIBDIR}/cmake/MonopodSDK
# )
