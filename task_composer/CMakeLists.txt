find_package(boost_plugin_loader REQUIRED)
find_package(yaml-cpp REQUIRED)

# Define compile-time default variables
if(MSVC)
  set(TESSERACT_TASK_COMPOSER_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/bin)
else()
  set(TESSERACT_TASK_COMPOSER_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/lib)
endif()

# Variable for task composer plugins
set(TASK_COMPOSER_PLUGINS "")

# Core
list(APPEND SUPPORTED_COMPONENTS task_composer)
add_subdirectory(core)

# Planning Tasks
option(TESSERACT_BUILD_TASK_COMPOSER_PLANNING "Build task composer planning components" ON)
if(TESSERACT_BUILD_TASK_COMPOSER_PLANNING)
  message("Build task composer planning components")
  list(APPEND SUPPORTED_COMPONENTS task_composer_planning)
  add_subdirectory(planning)
endif()

# Taskflow Executor
option(TESSERACT_BUILD_TASK_COMPOSER_TASKFLOW "Build task composer taskflow components" ON)
if(TESSERACT_BUILD_TASK_COMPOSER_TASKFLOW)
  message("Build task composer taskflow components")
  list(APPEND SUPPORTED_COMPONENTS task_composer_taskflow)
  add_subdirectory(taskflow)
endif()

# Add compiler definition to core so it can find all plugins produced
string(
  REPLACE ";"
          ":"
          TASK_COMPOSER_PLUGINS_STRING
          "${TASK_COMPOSER_PLUGINS}")
target_compile_definitions(task_composer PRIVATE TESSERACT_TASK_COMPOSER_PLUGINS="${TASK_COMPOSER_PLUGINS_STRING}")

# Examples
if(TESSERACT_ENABLE_EXAMPLES)
  message("Build task composer examples components")
  list(APPEND SUPPORTED_COMPONENTS task_composer_examples)
  add_subdirectory(examples)
endif()

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/task_composer/config COMPONENT task_composer)

# Testing
if(TESSERACT_ENABLE_TESTING)
  add_subdirectory(test)
endif()

set(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS} PARENT_SCOPE)
