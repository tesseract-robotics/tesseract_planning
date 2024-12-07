find_package(tesseract_command_language REQUIRED)

# Profile Dictionary Tests
add_executable(${PROJECT_NAME}_profile_dictionary_unit profile_dictionary_tests.cpp)
target_link_libraries(${PROJECT_NAME}_profile_dictionary_unit PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME}_core)
target_compile_options(${PROJECT_NAME}_profile_dictionary_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                       ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_profile_dictionary_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_profile_dictionary_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_profile_dictionary_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_profile_dictionary_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_profile_dictionary_unit)
add_dependencies(${PROJECT_NAME}_profile_dictionary_unit ${PROJECT_NAME}_core)
add_dependencies(run_tests ${PROJECT_NAME}_profile_dictionary_unit)
