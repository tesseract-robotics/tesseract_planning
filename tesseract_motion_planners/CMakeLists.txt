cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# All of these are used by all components so to avoid having them in each they are include her
find_package(console_bridge REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(tesseract REQUIRED COMPONENTS common)
find_package(tesseract REQUIRED COMPONENTS kinematics)
find_package(tesseract REQUIRED COMPONENTS environment)
find_package(tesseract_command_language REQUIRED)
find_package(cereal REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

initialize_code_coverage(ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
set(COVERAGE_EXCLUDE
    /usr/*
    /opt/*
    ${CMAKE_CURRENT_LIST_DIR}/core/test/*
    ${CMAKE_CURRENT_LIST_DIR}/descartes/test/*
    ${CMAKE_CURRENT_LIST_DIR}/ompl/test/*
    ${CMAKE_CURRENT_LIST_DIR}/simple/test/*
    ${CMAKE_CURRENT_LIST_DIR}/trajopt/test/*
    ${CMAKE_CURRENT_LIST_DIR}/trajopt_ifopt/test/*
    /*/install/*
    /*/devel/*
    /*/gtest/*
    /*/include/descartes_light/*
    /*/include/opw_kinematics/*
    /*/include/trajopt/*
    /*/include/trajopt_sco/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# Enable Testing
if(TESSERACT_ENABLE_TESTING)
  find_gtest()
  enable_testing()
endif()

# Core
set(SUPPORTED_COMPONENTS core)
add_subdirectory(core)

# Simple
list(APPEND SUPPORTED_COMPONENTS simple)
add_subdirectory(simple)

# OMPL
option(TESSERACT_BUILD_OMPL "Build the OMPL planner" ON)
if(TESSERACT_BUILD_OMPL)
  message("Building OMPL Planner")
  list(APPEND SUPPORTED_COMPONENTS ompl)
  add_subdirectory(ompl)
endif()

# Descartes
option(TESSERACT_BUILD_DESCARTES "Build the Descartes planner" ON)
if(TESSERACT_BUILD_DESCARTES)
  message("Building Descartes Planner")
  list(APPEND SUPPORTED_COMPONENTS descartes)
  add_subdirectory(descartes)
endif()

# Trajopt
option(TESSERACT_BUILD_TRAJOPT "Build the Trajopt planner" ON)
if(TESSERACT_BUILD_TRAJOPT)
  message("Building Trajopt Planner")
  list(APPEND SUPPORTED_COMPONENTS trajopt)
  add_subdirectory(trajopt)
endif()

# Trajopt IFOPT
option(TESSERACT_BUILD_TRAJOPT_IFOPT "Build the Trajopt IFOPT planner" ON)
if(TESSERACT_BUILD_TRAJOPT_IFOPT)
  message("Building Trajopt IFOPT Planner")
  list(APPEND SUPPORTED_COMPONENTS trajopt_ifopt)
  add_subdirectory(trajopt_ifopt)
endif()

# Examples
if(TESSERACT_ENABLE_EXAMPLES)
  add_subdirectory(examples)
endif()

# Package configuration
configure_package(COMPONENT core SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS})

if(TESSERACT_PACKAGE)
  cpack_component_package(
    VERSION ${pkg_extracted_version}
    MAINTAINER_NAME ${pkg_extracted_maintainer_name}
    MAINTAINER_EMAIL ${pkg_extracted_maintainer_email}
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    COMPONENT_DEPENDS ${SUPPORTED_COMPONENTS})

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
