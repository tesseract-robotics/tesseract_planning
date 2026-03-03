find_package(tesseract REQUIRED COMPONENTS environment visualization)
find_package(Taskflow REQUIRED)

add_executable(tesseract_task_composer_example task_composer_example.cpp)
target_link_libraries(
  tesseract_task_composer_example
  PRIVATE task_composer
          task_composer_taskflow
          console_bridge::console_bridge
          tesseract::environment
          tesseract::command_language)
target_compile_options(tesseract_task_composer_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                               ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_task_composer_example PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_task_composer_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_task_composer_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(tesseract_task_composer_example PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
list(APPEND Examples tesseract_task_composer_example)

add_executable(tesseract_task_composer_trajopt_example task_composer_trajopt_example.cpp)
target_link_libraries(
  tesseract_task_composer_trajopt_example
  PRIVATE task_composer
          task_composer_taskflow
          console_bridge::console_bridge
          tesseract::environment
          tesseract::command_language
          tesseract::visualization)
target_compile_options(tesseract_task_composer_trajopt_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                       ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_task_composer_trajopt_example PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_task_composer_trajopt_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_task_composer_trajopt_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(tesseract_task_composer_trajopt_example
                           PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
list(APPEND Examples tesseract_task_composer_trajopt_example)

add_executable(tesseract_task_composer_raster_example task_composer_raster_example.cpp)
target_link_libraries(
  tesseract_task_composer_raster_example
  PRIVATE task_composer
          task_composer_taskflow
          console_bridge::console_bridge
          tesseract::environment
          tesseract::command_language
          tesseract::visualization)
target_compile_options(tesseract_task_composer_raster_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                      ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(tesseract_task_composer_raster_example PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(tesseract_task_composer_raster_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(tesseract_task_composer_raster_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(tesseract_task_composer_raster_example
                           PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
list(APPEND Examples tesseract_task_composer_raster_example)

# Configure Components
configure_component(
  COMPONENT task_composer_examples
  NAMESPACE tesseract
  TARGETS ${Examples}
  DEPENDENCIES console_bridge "tesseract COMPONENTS environment visualization"
               "tesseract_planning COMPONENTS command_language")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT task_composer_examples
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract task composer examples components"
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-environment"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-command-language"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-visualization"
    WINDOWS_DEPENDS
      "console-bridge"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-environment"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-command-language"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-visualization")
endif()
