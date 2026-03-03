# Create interface for core
add_library(motion_planners src/planner.cpp src/types.cpp src/utils.cpp)
add_library(tesseract::motion_planners ALIAS motion_planners)
set_target_properties(motion_planners PROPERTIES OUTPUT_NAME tesseract_motion_planners)
target_link_libraries(
  motion_planners
  PUBLIC tesseract::environment
         tesseract::common
         tesseract::command_language
         console_bridge::console_bridge
         Eigen3::Eigen)
target_compile_options(motion_planners PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(motion_planners PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(motion_planners PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(motion_planners ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(motion_planners PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  motion_planners
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(motion_planners PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")

# Mark cpp header files for installation
install(DIRECTORY include/tesseract DESTINATION include COMPONENT motion_planners)

# Configure Components
configure_component(
  COMPONENT motion_planners
  NAMESPACE tesseract
  TARGETS motion_planners
  DEPENDENCIES
    "tesseract COMPONENTS environment common"
    "tesseract_planning COMPONENTS command_language"
    console_bridge
    Threads
    Boost)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT motion_planners
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract motion planners core components"
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libeigen3-dev"
      "libboost-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-planning-command-language"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-environment"
    WINDOWS_DEPENDS
      "console-bridge"
      "Eigen3"
      "boost"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-planning-command-language"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-environment")
endif()
