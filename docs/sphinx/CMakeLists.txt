
add_custom_target(apidoc ALL DEPENDS doxygen)

# =============
# Create built package for overlord webpage in /html
# =============
set(SPHINX_BUILD ${CMAKE_BINARY_DIR}/html)
set(SPHINX_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}")

# =============
# SPHINX TARGET
# =============

find_package(Sphinx REQUIRED)
find_package(SphinxMultiVersion REQUIRED)

if(${DOXYGEN_OUTPUT_DIRECTORY} STREQUAL "")
    message(FATAL_ERROR "Doxygen was not configured properly")
endif()

add_custom_target(image_copy ALL
    DEPENDS apidoc
    COMMAND ${CMAKE_COMMAND} -E copy "${SPHINX_SOURCE}/media/device_class_diagram.svg" "${DOXYGEN_OUTPUT_DIRECTORY}/xml/device_class_diagram.svg"
    COMMENT "copy device class device svg into doxygen xml."
  )

add_custom_target(source_rst_copy ALL
    DEPENDS apidoc
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${SPHINX_SOURCE}/monopod_sdk_docs" "${SPHINX_BUILD}/sphinx_build/monopod_sdk_docs"
    COMMENT "copy source doc build files."
  )

add_custom_target(index_copy ALL
    DEPENDS apidoc
    COMMAND ${CMAKE_COMMAND} -E copy "${SPHINX_SOURCE}/index.rst" "${SPHINX_BUILD}/sphinx_build/monopod_sdk_index.rst"
    COMMENT "copy index rst."
  )

add_custom_target(doxygen_xml_copy ALL
    DEPENDS apidoc
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${DOXYGEN_OUTPUT_DIRECTORY}/xml" "${SPHINX_BUILD}/sphinx_build/monopod_sdk_docs/xml"
    COMMENT "copy source doxygen xml build files."
  )

IF(NOT BUILD_DOCS_SUPERBUILD)
    # Extend the Python path including before generating the website
    # Sphinx build command
    list(APPEND SPHINX_CMD
      ${SPHINX_MULTIVERSION_EXECUTABLE}
      ${SPHINX_SOURCE} ${SPHINX_BUILD}
    -D breathe_projects.monopod_sdk="${DOXYGEN_OUTPUT_DIRECTORY}/xml")

    # Generate the website
    add_custom_target(sphinx ALL
      DEPENDS apidoc
      COMMAND ${SPHINX_CMD}
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Generating documentation with Sphinx")

    # Disable GitHub pages autogeneration
    add_custom_command(
      TARGET sphinx POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E touch "${SPHINX_BUILD}/.nojekyll"
    COMMENT "Disabling Jekyll in html folder")

    # Handle redirect
    add_custom_command(
      TARGET sphinx POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/index.html" "${SPHINX_BUILD}"
    COMMENT "Copying html redirect to html folder")
ENDIF()
