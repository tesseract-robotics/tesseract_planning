add_executable(${PROJECT_NAME}_time_optimal_trajectory_generation_tests time_optimal_trajectory_generation_tests.cpp)
target_link_libraries(${PROJECT_NAME}_time_optimal_trajectory_generation_tests PRIVATE GTest::GTest GTest::Main
                                                                                       ${PROJECT_NAME}_totg)
target_compile_options(${PROJECT_NAME}_time_optimal_trajectory_generation_tests
                       PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_time_optimal_trajectory_generation_tests
                       PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(${PROJECT_NAME}_time_optimal_trajectory_generation_tests ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_time_optimal_trajectory_generation_tests PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_time_optimal_trajectory_generation_tests
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_time_optimal_trajectory_generation_tests)
add_dependencies(${PROJECT_NAME}_time_optimal_trajectory_generation_tests ${PROJECT_NAME}_totg)
