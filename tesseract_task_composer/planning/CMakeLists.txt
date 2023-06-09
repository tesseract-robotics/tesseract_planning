find_package(tesseract_command_language REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_time_parameterization REQUIRED)
find_package(trajopt REQUIRED)

add_library(${PROJECT_NAME}_planning src/planning_task_composer_problem.cpp)
target_link_libraries(
  ${PROJECT_NAME}_planning
  PUBLIC ${PROJECT_NAME}
         console_bridge::console_bridge
         tesseract::tesseract_common
         tesseract::tesseract_command_language
         tesseract::tesseract_environment
         Boost::boost
         Boost::serialization
         yaml-cpp)
target_compile_options(${PROJECT_NAME}_planning PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_planning PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(${PROJECT_NAME}_planning ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_planning PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_planning
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_planning PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                           "$<INSTALL_INTERFACE:include>")

add_library(
  ${PROJECT_NAME}_planning_nodes
  src/nodes/check_input_task.cpp
  src/nodes/continuous_contact_check_task.cpp
  src/nodes/discrete_contact_check_task.cpp
  src/nodes/fix_state_bounds_task.cpp
  src/nodes/fix_state_collision_task.cpp
  src/nodes/format_as_input_task.cpp
  src/nodes/iterative_spline_parameterization_task.cpp
  src/nodes/profile_switch_task.cpp
  src/nodes/raster_motion_task.cpp
  src/nodes/raster_only_motion_task.cpp
  src/nodes/ruckig_trajectory_smoothing_task.cpp
  src/nodes/min_length_task.cpp
  src/nodes/motion_planner_task_info.cpp
  src/nodes/time_optimal_parameterization_task.cpp
  src/nodes/update_end_state_task.cpp
  src/nodes/update_start_and_end_state_task.cpp
  src/nodes/update_start_state_task.cpp
  src/nodes/upsample_trajectory_task.cpp)
target_link_libraries(
  ${PROJECT_NAME}_planning_nodes
  PUBLIC ${PROJECT_NAME}
         ${PROJECT_NAME}_nodes
         ${PROJECT_NAME}_planning
         console_bridge::console_bridge
         tesseract::tesseract_common
         tesseract::tesseract_command_language
         tesseract::tesseract_motion_planners_core
         tesseract::tesseract_motion_planners_simple
         tesseract::tesseract_motion_planners_trajopt
         tesseract::tesseract_motion_planners_ompl
         tesseract::tesseract_motion_planners_descartes
         tesseract::tesseract_time_parameterization_core
         tesseract::tesseract_time_parameterization_isp
         tesseract::tesseract_time_parameterization_ruckig
         tesseract::tesseract_time_parameterization_totg
         trajopt::trajopt
         Boost::boost
         Boost::serialization
         yaml-cpp)
target_compile_options(${PROJECT_NAME}_planning_nodes PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_planning_nodes PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_planning_nodes PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_planning_nodes ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_planning_nodes PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_planning_nodes
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_planning_nodes PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                        "$<INSTALL_INTERFACE:include>")

# Create target for Bullet implementation
add_library(${PROJECT_NAME}_planning_factories src/planning_task_composer_plugin_factories.cpp)
target_link_libraries(
  ${PROJECT_NAME}_planning_factories
  PUBLIC ${PROJECT_NAME}
         ${PROJECT_NAME}_planning_nodes
         tesseract::tesseract_motion_planners_simple
         tesseract::tesseract_motion_planners_trajopt
         tesseract::tesseract_motion_planners_ompl
         tesseract::tesseract_motion_planners_descartes)
target_compile_options(${PROJECT_NAME}_planning_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_planning_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_planning_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_planning_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_planning_factories ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  ${PROJECT_NAME}_planning_factories
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_planning_factories PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                            "$<INSTALL_INTERFACE:include>")
if(TESSERACT_BUILD_TRAJOPT_IFOPT)
  target_link_libraries(${PROJECT_NAME}_planning_factories PUBLIC tesseract::tesseract_motion_planners_trajopt_ifopt)
  target_compile_definitions(${PROJECT_NAME}_planning_factories PUBLIC TESSERACT_TASK_COMPOSER_HAS_TRAJOPT_IFOPT=1)
endif()

# Add factory library so task_composer_factory can find these factories by defauult
set(TASK_COMPOSER_PLUGINS ${TASK_COMPOSER_PLUGINS} "${PROJECT_NAME}_planning_factories" PARENT_SCOPE)

# Mark header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/planning
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

install_targets(
  TARGETS
  ${PROJECT_NAME}_planning
  ${PROJECT_NAME}_planning_nodes
  ${PROJECT_NAME}_planning_factories)
