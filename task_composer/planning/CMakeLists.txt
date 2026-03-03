# Need to create export header so static class variables can be exported on windows build
include(GenerateExportHeader)

# When building in-tree, sibling component targets are already available via add_subdirectory. Optional component
# detection uses TARGET checks below.
find_package(trajopt REQUIRED)

set(LIB_SOURCE_FILES
    src/cereal_serialization.cpp
    src/nodes/continuous_contact_check_task.cpp
    src/nodes/discrete_contact_check_task.cpp
    src/nodes/fix_state_bounds_task.cpp
    src/nodes/fix_state_collision_task.cpp
    src/nodes/format_as_input_task.cpp
    src/nodes/format_as_result_task.cpp
    src/nodes/format_planning_input_task.cpp
    src/nodes/kinematic_limits_check_task.cpp
    src/nodes/profile_switch_task.cpp
    src/nodes/min_length_task.cpp
    src/nodes/process_planning_input_task.cpp
    src/nodes/update_end_state_task.cpp
    src/nodes/update_start_and_end_state_task.cpp
    src/nodes/update_start_state_task.cpp
    src/nodes/upsample_trajectory_task.cpp
    src/nodes/raster_motion_task.cpp
    src/nodes/raster_only_motion_task.cpp
    src/profiles/contact_check_profile.cpp
    src/profiles/fix_state_bounds_profile.cpp
    src/profiles/fix_state_collision_profile.cpp
    src/profiles/kinematic_limits_check_profile.cpp
    src/profiles/min_length_profile.cpp
    src/profiles/profile_switch_profile.cpp
    src/profiles/upsample_trajectory_profile.cpp)

set(LIB_SOURCE_LINK_LIBRARIES
    tesseract::task_composer
    tesseract::task_composer_nodes
    console_bridge::console_bridge
    trajopt::trajopt
    tesseract::common
    tesseract::command_language
    tesseract::motion_planners
    tesseract::motion_planners_simple
    tesseract::motion_planners_trajopt
    tesseract::time_parameterization
    Boost::boost
    cereal::cereal
    yaml-cpp)

set(FACTORIES_SOURCE_FILES src/factories/planning_task_composer_core_plugin_factories.cpp)

set(FACTORIES_SOURCE_LINK_LIBRARIES tesseract::task_composer task_composer_planning_nodes
                                    tesseract::motion_planners_simple)

set(LINUX_DEPENDS
    "libconsole-bridge-dev"
    "cereal"
    "${TESSERACT_PACKAGE_PREFIX}trajopt"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-planning-command-language"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-planning-motion-planners-core"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-planning-motion-planners-simple"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-planning-time-parameterization-core")

set(WINDOWS_DEPENDS
    "console-bridge"
    "cereal"
    "${TESSERACT_PACKAGE_PREFIX}trajopt"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-planning-command-language"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-planning-motion-planners-core"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-planning-motion-planners-simple"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-planning-time-parameterization-core")

if(tesseract_planning_motion_planners_trajopt_FOUND OR TARGET tesseract::motion_planners_trajopt)
  list(APPEND FACTORIES_SOURCE_FILES src/factories/planning_task_composer_trajopt_plugin_factory.cpp)
  list(APPEND FACTORIES_SOURCE_LINK_LIBRARIES tesseract::motion_planners_trajopt)
  list(APPEND LIB_SOURCE_LINK_LIBRARIES tesseract::motion_planners_trajopt)
endif()

if(tesseract_planning_motion_planners_descartes_FOUND OR TARGET tesseract::motion_planners_descartes)
  list(APPEND FACTORIES_SOURCE_FILES src/factories/planning_task_composer_descartes_plugin_factory.cpp)
  list(APPEND FACTORIES_SOURCE_LINK_LIBRARIES tesseract::motion_planners_descartes)
endif()

if(tesseract_planning_motion_planners_ompl_FOUND OR TARGET tesseract::motion_planners_ompl)
  list(APPEND FACTORIES_SOURCE_FILES src/factories/planning_task_composer_ompl_plugin_factory.cpp)
  list(APPEND FACTORIES_SOURCE_LINK_LIBRARIES tesseract::motion_planners_ompl)
endif()

if(tesseract_planning_motion_planners_trajopt_ifopt_FOUND OR TARGET tesseract::motion_planners_trajopt_ifopt)
  list(APPEND FACTORIES_SOURCE_FILES src/factories/planning_task_composer_trajopt_ifopt_plugin_factory.cpp)
  list(APPEND FACTORIES_SOURCE_LINK_LIBRARIES tesseract::motion_planners_trajopt_ifopt)
endif()

if(tesseract_planning_time_parameterization_isp_FOUND OR TARGET tesseract::time_parameterization_isp)
  list(APPEND LIB_SOURCE_FILES src/nodes/iterative_spline_parameterization_task.cpp)
  list(APPEND LIB_SOURCE_LINK_LIBRARIES tesseract::time_parameterization_isp)
  list(APPEND FACTORIES_SOURCE_FILES src/factories/planning_task_composer_isp_plugin_factory.cpp)
  list(APPEND FACTORIES_SOURCE_LINK_LIBRARIES tesseract::time_parameterization_isp)
endif()

if(tesseract_planning_time_parameterization_kdl_FOUND OR TARGET tesseract::time_parameterization_kdl)
  list(APPEND LIB_SOURCE_FILES src/nodes/constant_tcp_speed_parameterization_task.cpp)
  list(APPEND LIB_SOURCE_LINK_LIBRARIES tesseract::time_parameterization_kdl)
  list(APPEND FACTORIES_SOURCE_FILES src/factories/planning_task_composer_kdl_plugin_factory.cpp)
  list(APPEND FACTORIES_SOURCE_LINK_LIBRARIES tesseract::time_parameterization_kdl)
endif()

if(tesseract_planning_time_parameterization_totg_FOUND OR TARGET tesseract::time_parameterization_totg)
  list(APPEND LIB_SOURCE_FILES src/nodes/time_optimal_parameterization_task.cpp)
  list(APPEND LIB_SOURCE_LINK_LIBRARIES tesseract::time_parameterization_totg)
  list(APPEND FACTORIES_SOURCE_FILES src/factories/planning_task_composer_totg_plugin_factory.cpp)
  list(APPEND FACTORIES_SOURCE_LINK_LIBRARIES tesseract::time_parameterization_totg)
endif()

if(tesseract_planning_time_parameterization_ruckig_FOUND OR TARGET tesseract::time_parameterization_ruckig)
  list(APPEND LIB_SOURCE_FILES src/nodes/ruckig_trajectory_smoothing_task.cpp)
  list(APPEND LIB_SOURCE_LINK_LIBRARIES tesseract::time_parameterization_ruckig)
  list(APPEND FACTORIES_SOURCE_FILES src/factories/planning_task_composer_ruckig_plugin_factory.cpp)
  list(APPEND FACTORIES_SOURCE_LINK_LIBRARIES tesseract::time_parameterization_ruckig)
endif()

add_library(task_composer_planning_nodes ${LIB_SOURCE_FILES})
add_library(tesseract::task_composer_planning_nodes ALIAS task_composer_planning_nodes)
set_target_properties(task_composer_planning_nodes PROPERTIES OUTPUT_NAME tesseract_task_composer_planning_nodes)
target_link_libraries(task_composer_planning_nodes PUBLIC ${LIB_SOURCE_LINK_LIBRARIES})
target_compile_options(task_composer_planning_nodes PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(task_composer_planning_nodes PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(task_composer_planning_nodes PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(task_composer_planning_nodes ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(task_composer_planning_nodes PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  task_composer_planning_nodes
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  task_composer_planning_nodes PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                      "$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>" "$<INSTALL_INTERFACE:include>")

generate_export_header(
  task_composer_planning_nodes
  BASE_NAME
  tesseract_task_composer_planning_nodes
  EXPORT_FILE_NAME
  ${PROJECT_BINARY_DIR}/tesseract/task_composer/planning/tesseract_task_composer_planning_nodes_export.h)

# Create factories target
add_library(task_composer_planning_factories ${FACTORIES_SOURCE_FILES})
set_target_properties(task_composer_planning_factories PROPERTIES OUTPUT_NAME
                                                                  tesseract_task_composer_planning_factories)
target_link_libraries(task_composer_planning_factories PUBLIC ${FACTORIES_SOURCE_LINK_LIBRARIES})
target_compile_options(task_composer_planning_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(task_composer_planning_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(task_composer_planning_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(task_composer_planning_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(task_composer_planning_factories ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  task_composer_planning_factories
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  task_composer_planning_factories PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                          "$<INSTALL_INTERFACE:include>")

# Add factory library so task_composer_factory can find these factories by default
set(TASK_COMPOSER_PLUGINS ${TASK_COMPOSER_PLUGINS} "tesseract_task_composer_planning_factories" PARENT_SCOPE)

# Mark header files for installation
install(
  DIRECTORY include/tesseract/task_composer/planning
  DESTINATION include/tesseract/task_composer
  COMPONENT task_composer_planning
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

install(FILES ${PROJECT_BINARY_DIR}/tesseract/task_composer/planning/tesseract_task_composer_planning_nodes_export.h
        DESTINATION include/tesseract/task_composer/planning/ COMPONENT task_composer_planning)

# Plugin should not be export only installed
install(
  TARGETS task_composer_planning_factories
  COMPONENT task_composer_planning
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# Configure Components
configure_component(
  COMPONENT task_composer_planning
  NAMESPACE tesseract
  TARGETS task_composer_planning_nodes
  DEPENDENCIES
    trajopt
    cereal
    "tesseract_planning COMPONENTS command_language motion_planners motion_planners_simple task_composer time_parameterization OPTIONAL_COMPONENTS motion_planners_descartes motion_planners_ompl motion_planners_trajopt motion_planners_trajopt_ifopt time_parameterization_isp time_parameterization_kdl time_parameterization_totg time_parameterization_ruckig"
)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT task_composer_planning
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract task composer planning components"
    COMPONENT_DEPENDS task_composer
    LINUX_DEPENDS ${LINUX_DEPENDS}
    WINDOWS_DEPENDS ${WINDOWS_DEPENDS})
endif()
