add_executable(motion_planners_trajopt_ifopt_yaml_unit trajopt_ifopt_planner_yaml_conversions_tests.cpp)
target_link_libraries(motion_planners_trajopt_ifopt_yaml_unit PRIVATE GTest::GTest GTest::Main
                                                                      motion_planners_trajopt_ifopt)
target_compile_options(motion_planners_trajopt_ifopt_yaml_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                       ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(motion_planners_trajopt_ifopt_yaml_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(motion_planners_trajopt_ifopt_yaml_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(motion_planners_trajopt_ifopt_yaml_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  motion_planners_trajopt_ifopt_yaml_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(motion_planners_trajopt_ifopt_yaml_unit)

add_dependencies(motion_planners_trajopt_ifopt_yaml_unit motion_planners_trajopt_ifopt)
