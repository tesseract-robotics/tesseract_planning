add_executable(time_parameterization_time_optimal_trajectory_generation_tests
               time_optimal_trajectory_generation_tests.cpp)
target_link_libraries(time_parameterization_time_optimal_trajectory_generation_tests PRIVATE GTest::GTest GTest::Main
                                                                                             time_parameterization_totg)
target_compile_options(time_parameterization_time_optimal_trajectory_generation_tests
                       PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(time_parameterization_time_optimal_trajectory_generation_tests
                       PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(time_parameterization_time_optimal_trajectory_generation_tests ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(time_parameterization_time_optimal_trajectory_generation_tests PRIVATE
                   VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  time_parameterization_time_optimal_trajectory_generation_tests
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(time_parameterization_time_optimal_trajectory_generation_tests)
add_dependencies(time_parameterization_time_optimal_trajectory_generation_tests time_parameterization_totg)
