cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(console_bridge REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_time_parameterization REQUIRED)
find_package(trajopt REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                  ${console_bridge_INCLUDE_DIRS})
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES ${console_bridge_LIBRARIES})
else()
  get_target_property(CHECK_INCLUDE_DIRECTORIES console_bridge::console_bridge INTERFACE_INCLUDE_DIRECTORIES)
  if(NOT ${CHECK_INCLUDE_DIRECTORIES})
    set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                    ${console_bridge_INCLUDE_DIRS})
  endif()
endif()

initialize_code_coverage()
set(COVERAGE_EXCLUDE
    /usr/*
    /opt/*
    ${CMAKE_CURRENT_LIST_DIR}/test/*
    /*/gtest/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE})

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

add_library(
  ${PROJECT_NAME}
  src/task_composer_data_storage.cpp
  src/task_composer_input.cpp
  src/task_composer_node.cpp
  src/task_composer_node_info.cpp
  src/task_composer_graph.cpp
  src/nodes/check_input_task.cpp
  src/nodes/continuous_contact_check_task.cpp
  src/nodes/discrete_contact_check_task.cpp
  src/nodes/done_task.cpp
  src/nodes/error_task.cpp
  src/nodes/fix_state_bounds_task.cpp
  src/nodes/fix_state_collision_task.cpp
  src/nodes/has_seed_task.cpp
  src/nodes/iterative_spline_parameterization_task.cpp
  src/nodes/motion_planner_task.cpp
  src/nodes/profile_switch_task.cpp
  src/nodes/ruckig_trajectory_smoothing_task.cpp
  src/nodes/seed_min_length_task.cpp
  src/nodes/time_optimal_parameterization_task.cpp
  src/nodes/transition_mux_task.cpp
  src/nodes/upsample_trajectory_task.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC console_bridge::console_bridge
         tesseract::tesseract_common
         tesseract::tesseract_command_language
         tesseract::tesseract_motion_planners_core
         tesseract::tesseract_time_parameterization
         trajopt::trajopt
         Boost::boost
         Boost::serialization)
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")

configure_package(NAMESPACE tesseract TARGETS ${PROJECT_NAME})

# Mark header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

if(TESSERACT_ENABLE_EXAMPLES)
  add_subdirectory(examples)
endif()

# if(TESSERACT_ENABLE_TESTING) enable_testing() add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
# add_subdirectory(test) endif()

if(TESSERACT_PACKAGE)
  tesseract_cpack(
    VERSION ${pkg_extracted_version}
    MAINTAINER <https://github.com/ros-industrial-consortium/tesseract>
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    LINUX_DEPENDS "libconsole-bridge-dev" "libboost-serialization-dev" "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
    WINDOWS_DEPENDS "console-bridge" "boost-serialization" "${TESSERACT_PACKAGE_PREFIX}tesseract-common")
endif()
