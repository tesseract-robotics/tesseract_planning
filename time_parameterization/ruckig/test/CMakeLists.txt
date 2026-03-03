add_executable(time_parameterization_ruckig_trajectory_smoothing_tests ruckig_trajectory_smoothing_tests.cpp)
target_link_libraries(
  time_parameterization_ruckig_trajectory_smoothing_tests
  PRIVATE GTest::GTest
          GTest::Main
          time_parameterization_ruckig
          time_parameterization_isp)
target_compile_options(time_parameterization_ruckig_trajectory_smoothing_tests
                       PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(time_parameterization_ruckig_trajectory_smoothing_tests
                       PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(time_parameterization_ruckig_trajectory_smoothing_tests ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(time_parameterization_ruckig_trajectory_smoothing_tests PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  time_parameterization_ruckig_trajectory_smoothing_tests
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(time_parameterization_ruckig_trajectory_smoothing_tests)
add_dependencies(time_parameterization_ruckig_trajectory_smoothing_tests time_parameterization_ruckig
                 time_parameterization_isp)
