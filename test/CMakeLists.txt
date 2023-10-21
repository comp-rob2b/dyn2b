find_package(check)

add_executable(main_test
  main_test.c
  vector3_test.c
  matrix_test.c
  screw_test.c
  mechanics_test.c
  joint_test.c
)

target_link_libraries(main_test
  PRIVATE
    dyn2b
    Check::checkShared
)

add_test(main_test
  ${CMAKE_CURRENT_BINARY_DIR}/main_test
)

if(ENABLE_TEST_COVERAGE)
  setup_target_for_coverage_lcov(
    NAME coverage
    DEPENDENCIES main_test
    EXECUTABLE main_test
  )
endif()