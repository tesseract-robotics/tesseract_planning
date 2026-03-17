find_gtest()

# Waypoint Tests
add_executable(command_language_unit command_language_unit.cpp)
target_link_libraries(command_language_unit PRIVATE GTest::GTest GTest::Main command_language)
target_compile_options(command_language_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                     ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(command_language_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(command_language_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  command_language_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(command_language_unit)
add_dependencies(command_language_unit command_language)

# Utils Tests
add_executable(command_language_utils_unit command_language_utils_unit.cpp)
target_link_libraries(command_language_utils_unit PRIVATE GTest::GTest GTest::Main command_language)
target_compile_options(command_language_utils_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                           ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(command_language_utils_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(command_language_utils_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  command_language_utils_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(command_language_utils_unit)
add_dependencies(command_language_utils_unit command_language)

# Type Erasure Benchmarks
find_package(benchmark REQUIRED)
add_executable(command_language_type_erasure_benchmark type_erasure_benchmark.cpp)
target_link_libraries(command_language_type_erasure_benchmark PRIVATE benchmark::benchmark command_language)
target_cxx_version(command_language_type_erasure_benchmark PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  command_language_type_erasure_benchmark
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
# add_run_benchmark_target(command_language_type_erasure_benchmark)
