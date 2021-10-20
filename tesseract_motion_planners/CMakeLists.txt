cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(console_bridge REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(Threads REQUIRED)

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
    /*/gtest/*
    /*/include/descartes_light/*
    /*/include/opw_kinematics/*
    /*/include/trajopt/*
    /*/include/trajopt_sco/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE})

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

add_subdirectory(core)

# OMPL
option(TESSERACT_BUILD_OMPL "Build the OMPL planner" ON)
if(TESSERACT_BUILD_OMPL)
  message("Building OMPL Planner")
  add_subdirectory(ompl)
endif()

# Descartes
option(TESSERACT_BUILD_DESCARTES "Build the Descartes planner" ON)
if(TESSERACT_BUILD_DESCARTES)
  message("Building Descartes Planner")
  add_subdirectory(descartes)
endif()

# Trajopt
option(TESSERACT_BUILD_TRAJOPT "Build the Trajopt planner" ON)
if(TESSERACT_BUILD_TRAJOPT)
  message("Building Trajopt Planner")
  add_subdirectory(trajopt)
endif()

# Trajopt IFOPT
option(TESSERACT_BUILD_TRAJOPT_IFOPT "Build the Trajopt IFOPT planner" ON)
if(TESSERACT_BUILD_TRAJOPT_IFOPT)
  message("Building Trajopt IFOPT Planner")
  add_subdirectory(trajopt_ifopt)
endif()

# Examples
add_subdirectory(examples)

# Tests
if(TESSERACT_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

# Package configuration
configure_package(NAMESPACE tesseract)
