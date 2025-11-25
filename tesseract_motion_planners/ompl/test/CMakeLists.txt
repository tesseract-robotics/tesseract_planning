find_package(tesseract_command_language REQUIRED)

add_executable(${PROJECT_NAME}_ompl_unit ompl_planner_tests.cpp)
target_link_libraries(${PROJECT_NAME}_ompl_unit PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME}_ompl)
target_compile_options(${PROJECT_NAME}_ompl_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                         ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_ompl_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_ompl_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_ompl_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_ompl_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_ompl_unit)
add_dependencies(${PROJECT_NAME}_ompl_unit ${PROJECT_NAME}_ompl)
add_dependencies(run_tests ${PROJECT_NAME}_ompl_unit)

# OMPL YAML Unit tests
add_executable(${PROJECT_NAME}_ompl_yaml_unit ompl_yaml_conversions_tests.cpp)
target_link_libraries(${PROJECT_NAME}_ompl_yaml_unit PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME}_ompl)
target_compile_options(${PROJECT_NAME}_ompl_yaml_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                              ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_ompl_yaml_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_ompl_yaml_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_ompl_yaml_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_ompl_yaml_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_ompl_yaml_unit)
add_dependencies(${PROJECT_NAME}_ompl_yaml_unit ${PROJECT_NAME}_ompl)
add_dependencies(run_tests ${PROJECT_NAME}_ompl_yaml_unit)

# OMPL Constrained Planning Test/Example Program if(NOT OMPL_VERSION VERSION_LESS "1.4.0")
# add_executable(${PROJECT_NAME}_ompl_constrained_unit ompl_constrained_planner_tests.cpp)
# target_link_libraries(${PROJECT_NAME}_ompl_constrained_unit PRIVATE Boost::boost Boost::system Boost::program_options
# GTest::GTest GTest::Main ${PROJECT_NAME}_ompl trajopt::trajopt)
# tesseract_target_compile_options(${PROJECT_NAME}_ompl_constrained_unit PRIVATE)
# tesseract_clang_tidy(${PROJECT_NAME}_ompl_constrained_unit)
# tesseract_code_coverage(${PROJECT_NAME}_ompl_constrained_unit ALL EXCLUDE ${COVERAGE_EXCLUDE})
# tesseract_gtest_discover_tests(${PROJECT_NAME}_ompl_constrained_unit)
# add_dependencies(${PROJECT_NAME}_ompl_constrained_unit ${PROJECT_NAME}_ompl) add_dependencies(run_tests
# ${PROJECT_NAME}_ompl_constrained_unit) endif()
