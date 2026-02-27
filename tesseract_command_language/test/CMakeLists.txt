find_package(GTest REQUIRED)

if(NOT TARGET GTest::GTest)
  add_library(GTest::GTest INTERFACE IMPORTED)
  set_target_properties(GTest::GTest PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}")
  if(${GTEST_LIBRARIES})
    set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_LIBRARIES}")
  else()
    if(MSVC)
      set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "gtest.lib")
    else()
      set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "libgtest.so")
    endif()
  endif()
endif()

if(NOT TARGET GTest::Main)
  add_library(GTest::Main INTERFACE IMPORTED)
  set_target_properties(GTest::Main PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}")
  if(${GTEST_MAIN_LIBRARIES})
    set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_MAIN_LIBRARIES}")
  else()
    if(MSVC)
      set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "gtest_main.lib")
    else()
      set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "libgtest_main.so")
    endif()
  endif()
endif()

include(GoogleTest)

# Waypoint Tests
add_executable(${PROJECT_NAME}_unit command_language_unit.cpp)
target_link_libraries(${PROJECT_NAME}_unit PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME})
target_compile_options(${PROJECT_NAME}_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                    ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(${PROJECT_NAME}_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_unit)
add_dependencies(${PROJECT_NAME}_unit ${PROJECT_NAME})

# Utils Tests
add_executable(${PROJECT_NAME}_utils_unit command_language_utils_unit.cpp)
target_link_libraries(${PROJECT_NAME}_utils_unit PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME})
target_compile_options(${PROJECT_NAME}_utils_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                          ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(${PROJECT_NAME}_utils_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_utils_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_utils_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_utils_unit)
add_dependencies(${PROJECT_NAME}_utils_unit ${PROJECT_NAME})

# Type Erasure Benchmarks
find_package(benchmark REQUIRED)
add_executable(${PROJECT_NAME}_type_erasure_benchmark type_erasure_benchmark.cpp)
target_link_libraries(${PROJECT_NAME}_type_erasure_benchmark PRIVATE benchmark::benchmark ${PROJECT_NAME})
target_cxx_version(${PROJECT_NAME}_type_erasure_benchmark PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_type_erasure_benchmark
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
# add_run_benchmark_target(${PROJECT_NAME}_type_erasure_benchmark)
