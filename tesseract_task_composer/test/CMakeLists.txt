find_package(GTest REQUIRED)
find_package(tesseract_support REQUIRED)
find_package(tesseract_kinematics REQUIRED)

if(NOT WIN32)
  find_package(tcmalloc_minimal REQUIRED)
endif()

find_gtest()

if(NOT WIN32)
  set(TESSERACT_TCMALLOC_LIB tcmalloc::tcmalloc_minimal)
endif()

add_executable(${PROJECT_NAME}_unit tesseract_task_composer_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::tesseract_support
          tesseract::tesseract_kinematics_opw
          ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_unit PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/examples>")
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
# add_gtest_discover_tests(${PROJECT_NAME}_unit)
add_dependencies(${PROJECT_NAME}_unit ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_unit)

# add_executable(${PROJECT_NAME}_fix_state_bounds_task_generator_unit fix_state_bounds_task_generator_unit.cpp)
# target_link_libraries( ${PROJECT_NAME}_fix_state_bounds_task_generator_unit PRIVATE GTest::GTest GTest::Main
# tesseract::tesseract_support ${PROJECT_NAME} ${TESSERACT_TCMALLOC_LIB})
# target_include_directories(${PROJECT_NAME}_fix_state_bounds_task_generator_unit PUBLIC
# "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/examples>")
# target_compile_options(${PROJECT_NAME}_fix_state_bounds_task_generator_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_fix_state_bounds_task_generator_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_fix_state_bounds_task_generator_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
# target_code_coverage( ${PROJECT_NAME}_fix_state_bounds_task_generator_unit PRIVATE ALL EXCLUDE ${COVERAGE_EXCLUDE}
# ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
# add_gtest_discover_tests(${PROJECT_NAME}_fix_state_bounds_task_generator_unit) add_dependencies(run_tests
# ${PROJECT_NAME}_fix_state_bounds_task_generator_unit)

# add_executable(${PROJECT_NAME}_fix_state_collision_task_generator_unit fix_state_collision_task_generator_unit.cpp)
# target_link_libraries( ${PROJECT_NAME}_fix_state_collision_task_generator_unit PRIVATE GTest::GTest GTest::Main
# tesseract::tesseract_support ${PROJECT_NAME} ${TESSERACT_TCMALLOC_LIB})
# target_include_directories(${PROJECT_NAME}_fix_state_collision_task_generator_unit PUBLIC
# "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/examples>")
# target_compile_options(${PROJECT_NAME}_fix_state_collision_task_generator_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_fix_state_collision_task_generator_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_fix_state_collision_task_generator_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
# target_code_coverage( ${PROJECT_NAME}_fix_state_collision_task_generator_unit PRIVATE ALL EXCLUDE ${COVERAGE_EXCLUDE}
# ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
# add_gtest_discover_tests(${PROJECT_NAME}_fix_state_collision_task_generator_unit) add_dependencies(run_tests
# ${PROJECT_NAME}_fix_state_collision_task_generator_unit)

# add_executable(${PROJECT_NAME}_graph_taskflow_unit graph_taskflow_unit.cpp) target_link_libraries(
# ${PROJECT_NAME}_graph_taskflow_unit PUBLIC ${PROJECT_NAME} GTest::GTest GTest::Main ${TESSERACT_TCMALLOC_LIB})
# target_include_directories(${PROJECT_NAME}_graph_taskflow_unit PRIVATE
# "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>") target_cxx_version(${PROJECT_NAME}_graph_taskflow_unit PRIVATE
# VERSION ${TESSERACT_CXX_VERSION}) target_clang_tidy(${PROJECT_NAME}_graph_taskflow_unit ENABLE
# ${TESSERACT_ENABLE_CLANG_TIDY}) target_code_coverage( ${PROJECT_NAME}_graph_taskflow_unit PRIVATE ALL EXCLUDE
# ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
# add_gtest_discover_tests(${PROJECT_NAME}_graph_taskflow_unit) add_dependencies(run_tests
# ${PROJECT_NAME}_graph_taskflow_unit)

# Serialize Tests add_executable(${PROJECT_NAME}_serialization_unit ${PROJECT_NAME}_serialization_unit.cpp)
# target_link_libraries(${PROJECT_NAME}_serialization_unit PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME})
# target_include_directories(${PROJECT_NAME}_serialization_unit PUBLIC
# "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/examples>") target_compile_options(${PROJECT_NAME}_serialization_unit PRIVATE
# ${TESSERACT_COMPILE_OPTIONS_PRIVATE} ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
# target_clang_tidy(${PROJECT_NAME}_serialization_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_serialization_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION}) target_code_coverage(
# ${PROJECT_NAME}_serialization_unit PRIVATE ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
# add_gtest_discover_tests(${PROJECT_NAME}_serialization_unit) add_dependencies(run_tests
# ${PROJECT_NAME}_serialization_unit) add_dependencies(${PROJECT_NAME}_serialization_unit ${PROJECT_NAME})
