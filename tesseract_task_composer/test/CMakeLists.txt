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

# add_executable(${PROJECT_NAME}_unit tesseract_task_composer_unit.cpp) target_link_libraries( ${PROJECT_NAME}_unit
# PRIVATE GTest::GTest GTest::Main tesseract::tesseract_support tesseract::tesseract_kinematics_opw ${PROJECT_NAME})
# target_include_directories(${PROJECT_NAME}_unit PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/examples>")
# target_compile_options(${PROJECT_NAME}_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
# ${TESSERACT_COMPILE_OPTIONS_PUBLIC}) target_clang_tidy(${PROJECT_NAME}_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION}) target_code_coverage(
# ${PROJECT_NAME}_unit PRIVATE ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
# add_gtest_discover_tests(${PROJECT_NAME}_unit) add_dependencies(${PROJECT_NAME}_unit ${PROJECT_NAME})
# add_dependencies(run_tests ${PROJECT_NAME}_unit)

add_executable(${PROJECT_NAME}_fix_state_bounds_task_unit fix_state_bounds_task_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_fix_state_bounds_task_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::tesseract_support
          ${PROJECT_NAME}_planning_nodes
          ${TESSERACT_TCMALLOC_LIB})
target_include_directories(${PROJECT_NAME}_fix_state_bounds_task_unit
                           PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/examples>")
target_compile_options(${PROJECT_NAME}_fix_state_bounds_task_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_fix_state_bounds_task_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_fix_state_bounds_task_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_fix_state_bounds_task_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_fix_state_bounds_task_unit)
add_dependencies(run_tests ${PROJECT_NAME}_fix_state_bounds_task_unit)

add_executable(${PROJECT_NAME}_fix_state_collision_task_unit fix_state_collision_task_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_fix_state_collision_task_unit
  PRIVATE GTest::GTest
          GTest::Main
          tesseract::tesseract_support
          ${PROJECT_NAME}_planning_nodes
          ${TESSERACT_TCMALLOC_LIB})
target_include_directories(${PROJECT_NAME}_fix_state_collision_task_unit
                           PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/examples>")
target_compile_options(${PROJECT_NAME}_fix_state_collision_task_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_fix_state_collision_task_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_fix_state_collision_task_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_fix_state_collision_task_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_fix_state_collision_task_unit)
add_dependencies(run_tests ${PROJECT_NAME}_fix_state_collision_task_unit)

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

# Plugin Factories Tests
add_executable(${PROJECT_NAME}_plugin_factories_unit ${PROJECT_NAME}_plugin_factories_unit.cpp)
target_link_libraries(${PROJECT_NAME}_plugin_factories_unit PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_plugin_factories_unit
                           PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/examples>")
target_compile_options(${PROJECT_NAME}_plugin_factories_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                     ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(${PROJECT_NAME}_plugin_factories_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_plugin_factories_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_plugin_factories_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_plugin_factories_unit)
add_dependencies(run_tests ${PROJECT_NAME}_plugin_factories_unit)
add_dependencies(${PROJECT_NAME}_plugin_factories_unit ${PROJECT_NAME})
if(TESSERACT_BUILD_TRAJOPT_IFOPT)
  target_compile_definitions(${PROJECT_NAME}_plugin_factories_unit PUBLIC TESSERACT_TASK_COMPOSER_HAS_TRAJOPT_IFOPT=1)
endif()

# Core Tests
add_executable(${PROJECT_NAME}_core_unit ${PROJECT_NAME}_core_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_core_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}
          ${PROJECT_NAME}_nodes)
target_include_directories(${PROJECT_NAME}_core_unit PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/examples>")
target_compile_options(${PROJECT_NAME}_core_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                         ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(${PROJECT_NAME}_core_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_core_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_core_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_core_unit)
add_dependencies(run_tests ${PROJECT_NAME}_core_unit)
add_dependencies(${PROJECT_NAME}_core_unit ${PROJECT_NAME})
