find_package(Taskflow REQUIRED)

add_library(task_composer_taskflow src/taskflow_task_composer_executor.cpp src/taskflow_task_composer_future.cpp)
add_library(tesseract::task_composer_taskflow ALIAS task_composer_taskflow)
set_target_properties(task_composer_taskflow PROPERTIES OUTPUT_NAME tesseract_task_composer_taskflow)
target_link_libraries(
  task_composer_taskflow
  PUBLIC tesseract::task_composer
         console_bridge::console_bridge
         tesseract::common
         Boost::boost
         Taskflow::Taskflow
         yaml-cpp)
target_compile_options(task_composer_taskflow PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(task_composer_taskflow PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(task_composer_taskflow PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(task_composer_taskflow ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(task_composer_taskflow PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  task_composer_taskflow
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(task_composer_taskflow PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:include>")

# Create target for factory implementation
add_library(task_composer_taskflow_factories src/taskflow_task_composer_plugin_factories.cpp)
set_target_properties(task_composer_taskflow_factories PROPERTIES OUTPUT_NAME
                                                                  tesseract_task_composer_taskflow_factories)
target_link_libraries(task_composer_taskflow_factories PUBLIC tesseract::task_composer_taskflow)
target_compile_options(task_composer_taskflow_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(task_composer_taskflow_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(task_composer_taskflow_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(task_composer_taskflow_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(task_composer_taskflow_factories ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  task_composer_taskflow_factories
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  task_composer_taskflow_factories PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                          "$<INSTALL_INTERFACE:include>")

# Add factory library so task_composer_factory can find these factories by default
set(TASK_COMPOSER_PLUGINS ${TASK_COMPOSER_PLUGINS} "tesseract_task_composer_taskflow_factories" PARENT_SCOPE)

# Mark header files for installation
install(
  DIRECTORY include/tesseract/task_composer/taskflow
  DESTINATION include/tesseract/task_composer
  COMPONENT task_composer_taskflow
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

# Plugin should not be export only installed
install(
  TARGETS task_composer_taskflow_factories
  COMPONENT task_composer_taskflow
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# Configure Components
configure_component(
  COMPONENT task_composer_taskflow
  NAMESPACE tesseract
  TARGETS task_composer_taskflow
  DEPENDENCIES "tesseract_planning COMPONENTS task_composer" Taskflow)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT task_composer_taskflow
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract task composer taskflow components"
    COMPONENT_DEPENDS task_composer
    LINUX_DEPENDS "taskflow | libtaskflow-cpp-dev | ${TESSERACT_PACKAGE_PREFIX}taskflow"
    WINDOWS_DEPENDS "taskflow | libtaskflow-cpp-dev | ${TESSERACT_PACKAGE_PREFIX}taskflow")
endif()
