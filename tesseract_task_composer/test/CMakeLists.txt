find_package(tesseract_support REQUIRED)
find_package(
  tesseract_motion_planners REQUIRED
  COMPONENTS core simple
  OPTIONAL_COMPONENTS
    descartes
    ompl
    trajopt
    trajopt_ifopt)
find_package(tesseract_time_parameterization REQUIRED COMPONENTS core OPTIONAL_COMPONENTS isp totg ruckig)
find_package(trajopt REQUIRED)

if(NOT WIN32)
  find_package(tcmalloc_minimal REQUIRED)
  set(TESSERACT_TCMALLOC_LIB tcmalloc::tcmalloc_minimal)
endif()

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
if(tesseract_motion_planners_trajopt_ifopt_FOUND)
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

# Taskflow Tests
if(TESSERACT_BUILD_TASK_COMPOSER_TASKFLOW)
  add_executable(${PROJECT_NAME}_taskflow_unit ${PROJECT_NAME}_taskflow_unit.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_taskflow_unit
    PRIVATE GTest::GTest
            GTest::Main
            ${PROJECT_NAME}
            ${PROJECT_NAME}_nodes
            ${PROJECT_NAME}_taskflow)
  target_include_directories(${PROJECT_NAME}_taskflow_unit PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/examples>")
  target_compile_options(${PROJECT_NAME}_taskflow_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                               ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_clang_tidy(${PROJECT_NAME}_taskflow_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${PROJECT_NAME}_taskflow_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_code_coverage(
    ${PROJECT_NAME}_taskflow_unit
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  add_gtest_discover_tests(${PROJECT_NAME}_taskflow_unit)
  add_dependencies(run_tests ${PROJECT_NAME}_taskflow_unit)
  add_dependencies(${PROJECT_NAME}_taskflow_unit ${PROJECT_NAME})
endif(TESSERACT_BUILD_TASK_COMPOSER_TASKFLOW)

# Planning Tests
if(TESSERACT_BUILD_TASK_COMPOSER_PLANNING)
  find_package(tesseract_motion_planners REQUIRED COMPONENTS core simple OPTIONAL_COMPONENTS trajopt)

  add_executable(${PROJECT_NAME}_planning_unit ${PROJECT_NAME}_planning_unit.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_planning_unit
    PRIVATE GTest::GTest
            GTest::Main
            ${PROJECT_NAME}
            ${PROJECT_NAME}_nodes
            ${PROJECT_NAME}_planning_nodes
            tesseract::tesseract_motion_planners_trajopt)
  target_include_directories(${PROJECT_NAME}_planning_unit PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/examples>")
  target_compile_options(${PROJECT_NAME}_planning_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                               ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_clang_tidy(${PROJECT_NAME}_planning_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${PROJECT_NAME}_planning_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_code_coverage(
    ${PROJECT_NAME}_planning_unit
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  add_gtest_discover_tests(${PROJECT_NAME}_planning_unit)
  add_dependencies(run_tests ${PROJECT_NAME}_planning_unit)
  add_dependencies(${PROJECT_NAME}_planning_unit ${PROJECT_NAME})
endif()
