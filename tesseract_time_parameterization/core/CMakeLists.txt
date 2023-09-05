add_library(${PROJECT_NAME}_core src/instructions_trajectory.cpp src/tesseract_common_trajectory.cpp)
target_link_libraries(
  ${PROJECT_NAME}_core
  PUBLIC tesseract::tesseract_common
         tesseract::tesseract_command_language
         console_bridge::console_bridge
         Eigen3::Eigen)
target_include_directories(${PROJECT_NAME}_core PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                       "$<INSTALL_INTERFACE:include>")
target_compile_options(${PROJECT_NAME}_core PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_core PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_core PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_core ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_core PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_core
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# Mark header files for installation
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include COMPONENT core)

# Configure Components
configure_component(
  COMPONENT core
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_core
  DEPENDENCIES
    tesseract_common
    tesseract_command_language
    console_bridge
    Eigen3
  CFG_EXTRAS cmake/core-extras.cmake)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT core
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract motion planners core components"
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libeigen3-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-command-language"
    WINDOWS_DEPENDS
      "console-bridge"
      "Eigen3"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-command-language")
endif()
