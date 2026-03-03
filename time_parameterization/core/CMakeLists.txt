add_library(time_parameterization src/instructions_trajectory.cpp src/time_parameterization.cpp src/utils.cpp)
add_library(tesseract::time_parameterization ALIAS time_parameterization)
set_target_properties(time_parameterization PROPERTIES OUTPUT_NAME tesseract_time_parameterization)
target_link_libraries(
  time_parameterization
  PUBLIC tesseract::common
         tesseract::kinematics
         tesseract::environment
         tesseract::command_language
         console_bridge::console_bridge
         Eigen3::Eigen)
target_include_directories(time_parameterization PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                        "$<INSTALL_INTERFACE:include>")
target_compile_options(time_parameterization PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(time_parameterization PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(time_parameterization PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(time_parameterization ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(time_parameterization PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  time_parameterization
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# Mark header files for installation
install(DIRECTORY include/tesseract DESTINATION include COMPONENT time_parameterization)

# Testing
if(TESSERACT_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# Configure Components
configure_component(
  COMPONENT time_parameterization
  NAMESPACE tesseract
  TARGETS time_parameterization
  DEPENDENCIES
    "tesseract COMPONENTS common kinematics environment"
    "tesseract_planning COMPONENTS command_language"
    console_bridge
    Eigen3)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT time_parameterization
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract time parameterization core components"
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libeigen3-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-kinematics-core"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-environment"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-planning-command-language"
    WINDOWS_DEPENDS
      "console-bridge"
      "Eigen3"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-kinematics-core"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-environment"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-planning-command-language")
endif()
