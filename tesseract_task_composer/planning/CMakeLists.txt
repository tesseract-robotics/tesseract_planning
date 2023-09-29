find_package(tesseract_command_language REQUIRED)
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
set(LIB_SOURCE_FILES
    src/nodes/check_input_task.cpp
    src/nodes/continuous_contact_check_task.cpp
    src/nodes/discrete_contact_check_task.cpp
    src/nodes/fix_state_bounds_task.cpp
    src/nodes/fix_state_collision_task.cpp
    src/nodes/format_as_input_task.cpp
    src/nodes/profile_switch_task.cpp
    src/nodes/min_length_task.cpp
    src/nodes/motion_planner_task_info.cpp
    src/nodes/process_planning_input_task.cpp
    src/nodes/update_end_state_task.cpp
    src/nodes/update_start_and_end_state_task.cpp
    src/nodes/update_start_state_task.cpp
    src/nodes/upsample_trajectory_task.cpp
    src/nodes/raster_motion_task.cpp
    src/nodes/raster_only_motion_task.cpp)

set(LIB_SOURCE_LINK_LIBRARIES
    ${PROJECT_NAME}
    ${PROJECT_NAME}_nodes
    ${PROJECT_NAME}_planning
    console_bridge::console_bridge
    trajopt::trajopt
    tesseract::tesseract_common
    tesseract::tesseract_command_language
    tesseract::tesseract_motion_planners_core
    tesseract::tesseract_motion_planners_simple
    tesseract::tesseract_time_parameterization_core
    Boost::boost
    Boost::serialization
    yaml-cpp)

set(FACTORIES_SOURCE_FILES src/factories/planning_task_composer_core_plugin_factories.cpp)

set(FACTORIES_SOURCE_LINK_LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_planning_nodes
                                    tesseract::tesseract_motion_planners_simple)

set(LINUX_DEPENDS
    "libconsole-bridge-dev"
    "libboost-serialization-dev"
    "${TESSERACT_PACKAGE_PREFIX}trajopt"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-command-language"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-core"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-simple"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-time-parameterizaton-core")

set(WINDOWS_DEPENDS
    "console-bridge"
    "boost-serialization"
    "${TESSERACT_PACKAGE_PREFIX}trajopt"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-command-language"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-core"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-simple"
    "${TESSERACT_PACKAGE_PREFIX}tesseract-time-parameterizaton-core")

if(tesseract_motion_planners_trajopt_FOUND)
  list(APPEND FACTORIES_SOURCE_FILES src/factories/planning_task_composer_trajopt_plugin_factory.cpp)
  list(APPEND FACTORIES_SOURCE_LINK_LIBRARIES tesseract::tesseract_motion_planners_trajopt)
  list(APPEND LINUX_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-trajopt")
  list(APPEND WINDOWS_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-trajopt")
endif()

if(tesseract_motion_planners_descartes_FOUND)
  list(APPEND FACTORIES_SOURCE_FILES src/factories/planning_task_composer_descartes_plugin_factory.cpp)
  list(APPEND FACTORIES_SOURCE_LINK_LIBRARIES tesseract::tesseract_motion_planners_descartes)
  list(APPEND LINUX_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-descartes")
  list(APPEND WINDOWS_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-descartes")
endif()

if(tesseract_motion_planners_ompl_FOUND)
  list(APPEND FACTORIES_SOURCE_FILES src/factories/planning_task_composer_ompl_plugin_factory.cpp)
  list(APPEND FACTORIES_SOURCE_LINK_LIBRARIES tesseract::tesseract_motion_planners_ompl)
  list(APPEND LINUX_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-ompl")
  list(APPEND WINDOWS_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-ompl")
endif()

if(tesseract_motion_planners_trajopt_ifopt_FOUND)
  list(APPEND FACTORIES_SOURCE_FILES src/factories/planning_task_composer_trajopt_ifopt_plugin_factory.cpp)
  list(APPEND FACTORIES_SOURCE_LINK_LIBRARIES tesseract::tesseract_motion_planners_trajopt_ifopt)
  list(APPEND LINUX_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-trajopt-ifopt")
  list(APPEND WINDOWS_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-trajopt-ifopt")
endif()

if(tesseract_time_parameterization_isp_FOUND)
  list(APPEND LIB_SOURCE_FILES src/nodes/iterative_spline_parameterization_task.cpp)
  list(APPEND LIB_SOURCE_LINK_LIBRARIES tesseract::tesseract_time_parameterization_isp)
  list(APPEND FACTORIES_SOURCE_FILES src/factories/planning_task_composer_isp_plugin_factory.cpp)
  list(APPEND FACTORIES_SOURCE_LINK_LIBRARIES tesseract::tesseract_time_parameterization_isp)
  list(APPEND LINUX_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-time-parameterization-isp")
  list(APPEND WINDOWS_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-time-parameterization-isp")
endif()

if(tesseract_time_parameterization_totg_FOUND)
  list(APPEND LIB_SOURCE_FILES src/nodes/time_optimal_parameterization_task.cpp)
  list(APPEND LIB_SOURCE_LINK_LIBRARIES tesseract::tesseract_time_parameterization_totg)
  list(APPEND FACTORIES_SOURCE_FILES src/factories/planning_task_composer_totg_plugin_factory.cpp)
  list(APPEND FACTORIES_SOURCE_LINK_LIBRARIES tesseract::tesseract_time_parameterization_totg)
  list(APPEND LINUX_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-time-parameterization-totg")
  list(APPEND WINDOWS_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-time-parameterization-totg")
endif()

if(tesseract_time_parameterization_ruckig_FOUND)
  list(APPEND LIB_SOURCE_FILES src/nodes/ruckig_trajectory_smoothing_task.cpp)
  list(APPEND LIB_SOURCE_LINK_LIBRARIES tesseract::tesseract_time_parameterization_ruckig)
  list(APPEND FACTORIES_SOURCE_FILES src/factories/planning_task_composer_ruckig_plugin_factory.cpp)
  list(APPEND FACTORIES_SOURCE_LINK_LIBRARIES tesseract::tesseract_time_parameterization_ruckig)
  list(APPEND LINUX_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-time-parameterization-ruckig")
  list(APPEND WINDOWS_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-time-parameterization-ruckig")
endif()

add_library(${PROJECT_NAME}_planning_nodes ${LIB_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}_planning_nodes PUBLIC ${LIB_SOURCE_LINK_LIBRARIES})
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

# Create factories target
add_library(${PROJECT_NAME}_planning_factories ${FACTORIES_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}_planning_factories PUBLIC ${FACTORIES_SOURCE_LINK_LIBRARIES})
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

# Add factory library so task_composer_factory can find these factories by defauult
set(TASK_COMPOSER_PLUGINS ${TASK_COMPOSER_PLUGINS} "${PROJECT_NAME}_planning_factories" PARENT_SCOPE)

# Mark header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/planning
  DESTINATION include/${PROJECT_NAME}
  COMPONENT planning
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

# Configure Components
configure_component(
  COMPONENT planning
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_planning ${PROJECT_NAME}_planning_nodes ${PROJECT_NAME}_planning_factories
  DEPENDENCIES
    "tesseract_task_composer COMPONENTS core"
    tesseract_command_language
    trajopt
    "tesseract_motion_planners REQUIRED COMPONENTS core simple OPTIONAL_COMPONENTS descartes ompl trajopt trajopt_ifopt"
    "tesseract_time_parameterization REQUIRED COMPONENTS core OPTIONAL_COMPONENTS isp totg ruckig")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT planning
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract task composer planning components"
    COMPONENT_DEPENDS core
    LINUX_DEPENDS ${LINUX_DEPENDS}
    WINDOWS_DEPENDS ${WINDOWS_DEPENDS})
endif()
