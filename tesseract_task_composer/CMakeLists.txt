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
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(yaml-cpp REQUIRED)

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

# Define compile-time default variables
if(MSVC)
  set(TESSERACT_TASK_COMPOSER_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/bin)
else()
  set(TESSERACT_TASK_COMPOSER_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/lib)
endif()

option(TESSERACT_BUILD_TRAJOPT_IFOPT "Build the Trajopt IFOPT planner" ON)

# Variable for task composer plugins
set(TASK_COMPOSER_PLUGINS "")

# Core
add_subdirectory(core)

# Planning Tasks
option(TESSERACT_BUILD_TASK_COMPOSER_PLANNING "Build task composer planning components" ON)
if(TESSERACT_BUILD_TASK_COMPOSER_PLANNING)
  message("Build task composer planning components")
  add_subdirectory(planning)
endif()

# Taskflow Executor
option(TESSERACT_BUILD_TASK_COMPOSER_TASKFLOW "Build task composer taskflow components" ON)
if(TESSERACT_BUILD_TASK_COMPOSER_TASKFLOW)
  message("Build task composer taskflow components")
  add_subdirectory(taskflow)
endif()

# Add compiler definition to core so it can find all plugins produced
string(
  REPLACE ";"
          ":"
          TASK_COMPOSER_PLUGINS_STRING
          "${TASK_COMPOSER_PLUGINS}")
target_compile_definitions(${PROJECT_NAME} PRIVATE TESSERACT_TASK_COMPOSER_PLUGINS="${TASK_COMPOSER_PLUGINS_STRING}")

# Testing
if(TESSERACT_ENABLE_TESTING AND TESSERACT_BUILD_TASK_COMPOSER_PLANNING AND TESSERACT_BUILD_TASK_COMPOSER_TASKFLOW)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

# Examples
if(TESSERACT_ENABLE_EXAMPLES)
  add_subdirectory(examples)
endif()

configure_package(NAMESPACE tesseract)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

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
