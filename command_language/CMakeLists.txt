set(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS} "command_language" PARENT_SCOPE)

add_library(
  command_language
  src/poly/cartesian_waypoint_poly.cpp
  src/poly/instruction_poly.cpp
  src/poly/joint_waypoint_poly.cpp
  src/poly/move_instruction_poly.cpp
  src/poly/state_waypoint_poly.cpp
  src/poly/waypoint_poly.cpp
  src/cereal_serialization.cpp
  src/move_instruction.cpp
  src/set_analog_instruction.cpp
  src/set_digital_instruction.cpp
  src/set_tool_instruction.cpp
  src/timer_instruction.cpp
  src/wait_instruction.cpp
  src/composite_instruction.cpp
  src/instruction_type.cpp
  src/state_waypoint.cpp
  src/cartesian_waypoint.cpp
  src/joint_waypoint.cpp
  src/utils.cpp)
add_library(tesseract::command_language ALIAS command_language)
set_target_properties(command_language PROPERTIES OUTPUT_NAME tesseract_command_language)
target_link_libraries(
  command_language
  PUBLIC Eigen3::Eigen
         console_bridge::console_bridge
         tesseract::common
         Boost::boost
         cereal::cereal)
target_compile_options(command_language PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(command_language PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(command_language PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(command_language ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(command_language PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  command_language
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(command_language PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                   "$<INSTALL_INTERFACE:include>")

configure_component(
  COMPONENT command_language
  NAMESPACE tesseract
  TARGETS command_language
  DEPENDENCIES
    console_bridge
    "tesseract COMPONENTS common"
    Eigen3
    Boost
    cereal)

# Mark header files for installation
install(
  DIRECTORY include/tesseract
  DESTINATION include
  COMPONENT command_language
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

if(TESSERACT_ENABLE_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT command_language
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract command language components"
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libeigen3-dev"
      "cereal"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
    WINDOWS_DEPENDS
      "console-bridge"
      "Eigen3"
      "cereal"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common")
endif()
