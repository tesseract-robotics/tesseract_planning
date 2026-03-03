cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(console_bridge REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tesseract REQUIRED COMPONENTS common)
find_package(cereal REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

initialize_code_coverage(ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
set(COVERAGE_EXCLUDE
    /usr/*
    /opt/*
    ${CMAKE_CURRENT_LIST_DIR}/command_language/test/*
    ${CMAKE_CURRENT_LIST_DIR}/motion_planners/core/test/*
    ${CMAKE_CURRENT_LIST_DIR}/motion_planners/descartes/test/*
    ${CMAKE_CURRENT_LIST_DIR}/motion_planners/ompl/test/*
    ${CMAKE_CURRENT_LIST_DIR}/motion_planners/simple/test/*
    ${CMAKE_CURRENT_LIST_DIR}/motion_planners/trajopt/test/*
    ${CMAKE_CURRENT_LIST_DIR}/motion_planners/trajopt_ifopt/test/*
    ${CMAKE_CURRENT_LIST_DIR}/time_parameterization/core/test/*
    ${CMAKE_CURRENT_LIST_DIR}/time_parameterization/isp/test/*
    ${CMAKE_CURRENT_LIST_DIR}/time_parameterization/kdl/test/*
    ${CMAKE_CURRENT_LIST_DIR}/time_parameterization/ruckig/test/*
    ${CMAKE_CURRENT_LIST_DIR}/time_parameterization/totg/test/*
    ${CMAKE_CURRENT_LIST_DIR}/task_composer/test/*
    ${CMAKE_CURRENT_LIST_DIR}/task_composer/core/include/tesseract/task_composer/test_suite/*
    /*/install/*
    /*/devel/*
    /*/gtest/*
    /*/include/descartes_light/*
    /*/include/opw_kinematics/*
    /*/include/trajopt/*
    /*/include/trajopt_sco/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

set(SUPPORTED_COMPONENTS "")

if(TESSERACT_ENABLE_TESTING)
  find_gtest()
  enable_testing()
endif()

# Add subdirectories in dependency order
add_subdirectory(command_language)
add_subdirectory(motion_planners)
add_subdirectory(time_parameterization)
add_subdirectory(task_composer)
add_subdirectory(examples)

configure_package(COMPONENT command_language SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS})

if(TESSERACT_PACKAGE)
  cpack_component_package(
    VERSION ${pkg_extracted_version}
    MAINTAINER_NAME ${pkg_extracted_maintainer_name}
    MAINTAINER_EMAIL ${pkg_extracted_maintainer_email}
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/README.md
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
