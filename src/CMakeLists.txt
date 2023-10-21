# CMake removes the rpath from libraries/executables during installation. Thus,
# the installed libraries/executables will only search in system directories for
# linked libraries. CMake's behaviour can be overwritten by setting the
# CMAKE_INSTALL_RPATH variable.
# RPATH in CMake:
# - https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling
# - https://cmake.org/cmake/help/latest/variable/CMAKE_INSTALL_RPATH.html
# - https://cmake.org/cmake/help/latest/variable/CMAKE_INSTALL_RPATH_USE_LINK_PATH.html
# - https://stackoverflow.com/questions/32047805/whats-the-difference-between-cmake-install-prefix-and-cmake-install-rpath

list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_FULL_LIBDIR}" is_system_dir)
if("${is_system_dir}" STREQUAL "-1")
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_FULL_LIBDIR}")
endif()

find_package(cblas REQUIRED)
find_library(MATH_LIBRARY m)

add_library(dyn2b SHARED
  array.c
  vector3.c
  matrix.c
  screw.c
)

target_include_directories(dyn2b
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(dyn2b
  PRIVATE
    cblas
    ${MATH_LIBRARY}
)

set_target_properties(dyn2b
  PROPERTIES
    C_STANDARD 11
)

install(
  TARGETS dyn2b
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
