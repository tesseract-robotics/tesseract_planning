find_package(trajopt REQUIRED)

if(NOT WIN32)
  find_package(tcmalloc_minimal REQUIRED)
  set(TESSERACT_TCMALLOC_LIB tcmalloc::tcmalloc_minimal)
endif()

add_executable(tesseract_task_composer_fix_state_bounds_task_unit fix_state_bounds_task_unit.cpp)
target_link_libraries(
  tesseract_task_composer_fix_state_bounds_task_unit
  PRIVATE GTest::GTest
          GTest::Main
          task_composer_planning_nodes
          ${TESSERACT_TCMALLOC_LIB})
target_include_directories(tesseract_task_composer_fix_state_bounds_task_unit
                           PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/examples>")
target_compile_options(tesseract_task_composer_fix_state_bounds_task_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(tesseract_task_composer_fix_state_bounds_task_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_task_composer_fix_state_bounds_task_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_task_composer_fix_state_bounds_task_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_task_composer_fix_state_bounds_task_unit)

add_executable(tesseract_task_composer_fix_state_collision_task_unit fix_state_collision_task_unit.cpp)
target_link_libraries(
  tesseract_task_composer_fix_state_collision_task_unit
  PRIVATE GTest::GTest
          GTest::Main
          task_composer_planning_nodes
          ${TESSERACT_TCMALLOC_LIB})
target_include_directories(tesseract_task_composer_fix_state_collision_task_unit
                           PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/examples>")
target_compile_options(tesseract_task_composer_fix_state_collision_task_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(tesseract_task_composer_fix_state_collision_task_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_task_composer_fix_state_collision_task_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_task_composer_fix_state_collision_task_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_task_composer_fix_state_collision_task_unit)

# Plugin Factories Tests
add_executable(tesseract_task_composer_plugin_factories_unit tesseract_task_composer_plugin_factories_unit.cpp)
target_link_libraries(tesseract_task_composer_plugin_factories_unit PRIVATE GTest::GTest GTest::Main task_composer)
target_include_directories(tesseract_task_composer_plugin_factories_unit
                           PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/examples>")
target_compile_options(tesseract_task_composer_plugin_factories_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                             ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(tesseract_task_composer_plugin_factories_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_task_composer_plugin_factories_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_task_composer_plugin_factories_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_task_composer_plugin_factories_unit)
add_dependencies(tesseract_task_composer_plugin_factories_unit task_composer)
if(TARGET tesseract::motion_planners_trajopt_ifopt)
  target_compile_definitions(tesseract_task_composer_plugin_factories_unit
                             PUBLIC TESSERACT_TASK_COMPOSER_HAS_TRAJOPT_IFOPT=1)
endif()

# Core Tests
add_executable(tesseract_task_composer_core_unit tesseract_task_composer_core_unit.cpp)
target_link_libraries(
  tesseract_task_composer_core_unit
  PRIVATE GTest::GTest
          GTest::Main
          task_composer
          task_composer_nodes)
target_include_directories(tesseract_task_composer_core_unit PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/examples>")
target_compile_options(tesseract_task_composer_core_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                 ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(tesseract_task_composer_core_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_task_composer_core_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  tesseract_task_composer_core_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(tesseract_task_composer_core_unit)
add_dependencies(tesseract_task_composer_core_unit task_composer)

# Taskflow Tests
if(TESSERACT_BUILD_TASK_COMPOSER_TASKFLOW)
  add_executable(tesseract_task_composer_taskflow_unit tesseract_task_composer_taskflow_unit.cpp)
  target_link_libraries(
    tesseract_task_composer_taskflow_unit
    PRIVATE GTest::GTest
            GTest::Main
            task_composer
            task_composer_nodes
            task_composer_taskflow)
  target_include_directories(tesseract_task_composer_taskflow_unit
                             PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/examples>")
  target_compile_options(tesseract_task_composer_taskflow_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                       ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_clang_tidy(tesseract_task_composer_taskflow_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(tesseract_task_composer_taskflow_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_code_coverage(
    tesseract_task_composer_taskflow_unit
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  add_gtest_discover_tests(tesseract_task_composer_taskflow_unit)
  add_dependencies(tesseract_task_composer_taskflow_unit task_composer)
endif(TESSERACT_BUILD_TASK_COMPOSER_TASKFLOW)

# Planning Tests
if(TESSERACT_BUILD_TASK_COMPOSER_PLANNING)

  add_executable(tesseract_task_composer_planning_unit tesseract_task_composer_planning_unit.cpp)
  target_link_libraries(
    tesseract_task_composer_planning_unit
    PRIVATE GTest::GTest
            GTest::Main
            task_composer
            task_composer_nodes
            task_composer_planning_nodes
            tesseract::motion_planners_trajopt)
  target_include_directories(tesseract_task_composer_planning_unit
                             PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/examples>")
  target_compile_options(tesseract_task_composer_planning_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                       ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_clang_tidy(tesseract_task_composer_planning_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(tesseract_task_composer_planning_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_code_coverage(
    tesseract_task_composer_planning_unit
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  add_gtest_discover_tests(tesseract_task_composer_planning_unit)
  add_dependencies(tesseract_task_composer_planning_unit task_composer)
endif()
