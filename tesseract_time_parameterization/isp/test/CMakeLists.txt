add_executable(${PROJECT_NAME}_iterative_spline iterative_spline_tests.cpp)
target_link_libraries(${PROJECT_NAME}_iterative_spline PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME}_isp)
target_compile_options(${PROJECT_NAME}_iterative_spline PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_iterative_spline PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(${PROJECT_NAME}_iterative_spline ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_iterative_spline PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_iterative_spline
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_iterative_spline)
add_dependencies(${PROJECT_NAME}_iterative_spline ${PROJECT_NAME}_isp)
