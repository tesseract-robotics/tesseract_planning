add_executable(motion_planners_ompl_unit ompl_planner_tests.cpp)
target_link_libraries(motion_planners_ompl_unit PRIVATE GTest::GTest GTest::Main motion_planners_ompl)
target_compile_options(motion_planners_ompl_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                         ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(motion_planners_ompl_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(motion_planners_ompl_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(motion_planners_ompl_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  motion_planners_ompl_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(motion_planners_ompl_unit)
add_dependencies(motion_planners_ompl_unit motion_planners_ompl)

# OMPL YAML Unit tests
add_executable(motion_planners_ompl_yaml_unit ompl_yaml_conversions_tests.cpp)
target_link_libraries(motion_planners_ompl_yaml_unit PRIVATE GTest::GTest GTest::Main motion_planners_ompl)
target_compile_options(motion_planners_ompl_yaml_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                              ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(motion_planners_ompl_yaml_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(motion_planners_ompl_yaml_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(motion_planners_ompl_yaml_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  motion_planners_ompl_yaml_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(motion_planners_ompl_yaml_unit)
add_dependencies(motion_planners_ompl_yaml_unit motion_planners_ompl)

# OMPL Constrained Planning Test/Example Program if(NOT OMPL_VERSION VERSION_LESS "1.4.0")
# add_executable(motion_planners_ompl_constrained_unit ompl_constrained_planner_tests.cpp)
# target_link_libraries(motion_planners_ompl_constrained_unit PRIVATE Boost::boost Boost::system Boost::program_options
# GTest::GTest GTest::Main motion_planners_ompl trajopt::trajopt)
# tesseract_target_compile_options(motion_planners_ompl_constrained_unit PRIVATE)
# tesseract_clang_tidy(motion_planners_ompl_constrained_unit)
# tesseract_code_coverage(motion_planners_ompl_constrained_unit ALL EXCLUDE ${COVERAGE_EXCLUDE})
# tesseract_gtest_discover_tests(motion_planners_ompl_constrained_unit)
# add_dependencies(motion_planners_ompl_constrained_unit motion_planners_ompl) endif()
