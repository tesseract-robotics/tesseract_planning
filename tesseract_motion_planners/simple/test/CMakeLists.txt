find_package(tesseract_command_language REQUIRED)

macro(add_gtest test_name test_file)
  add_executable(${test_name} ${test_file})
  target_link_libraries(${test_name} PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME}_simple)
  target_compile_options(${test_name} PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE} ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${test_name} PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(${test_name} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${test_name} PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_code_coverage(
    ${test_name}
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  add_gtest_discover_tests(${test_name})
  add_dependencies(${test_name} ${PROJECT_NAME}_simple)
  add_dependencies(run_tests ${test_name})
endmacro()

add_gtest(${PROJECT_NAME}_simple_planner_fixed_size_assign_no_ik_plan_unit
          simple_planner_fixed_size_assign_no_ik_plan_unit.cpp)
add_gtest(${PROJECT_NAME}_simple_planner_fixed_size_plan_unit simple_planner_fixed_size_plan_unit.cpp)
add_gtest(${PROJECT_NAME}_simple_planner_lvs_plan_unit simple_planner_lvs_plan_unit.cpp)
