cmake_minimum_required(VERSION 3.10)
project(dyn2b)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/thirdparty/cmake/codecoverage)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/thirdparty/cmake/sphinx)

include(GNUInstallDirs)
include(CMakeDependentOption)

option(ENABLE_PACKAGE_REGISTRY              "Add this package to CMake's package registry" Off)

# The directory to which all CMake files (e.g. the configuration files) are installed
set(CMAKE_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

add_subdirectory(src)

# Export this package to CMake's package registry
if(ENABLE_PACKAGE_REGISTRY)
  export(PACKAGE ${PROJECT_NAME})
endif()

# Generate the CMake config file
include(CMakePackageConfigHelpers)
configure_package_config_file(${PROJECT_NAME}-config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_DIR}
)

# Generate the version file accompanying the project's config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  VERSION "0.1.0"
  COMPATIBILITY AnyNewerVersion
)

# Install cmake configuration and package version
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION ${CMAKE_INSTALL_DIR}
)

# Make the targets accessible from this packages's build tree
export(
  EXPORT ${PROJECT_NAME}-targets
  NAMESPACE ${PROJECT_NAME}::
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake"
)

# Make the targets accessible from this packages's install tree
install(
  EXPORT ${PROJECT_NAME}-targets
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_DIR}
)

# Install all public header files
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.h"
)
