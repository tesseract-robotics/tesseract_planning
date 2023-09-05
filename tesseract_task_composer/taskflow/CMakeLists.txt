find_package(Taskflow REQUIRED)

add_library(${PROJECT_NAME}_taskflow src/taskflow_task_composer_executor.cpp src/taskflow_task_composer_future.cpp)
target_link_libraries(
  ${PROJECT_NAME}_taskflow
  PUBLIC ${PROJECT_NAME}
         console_bridge::console_bridge
         tesseract::tesseract_common
         Boost::boost
         Boost::serialization
         Taskflow::Taskflow
         yaml-cpp)
target_compile_options(${PROJECT_NAME}_taskflow PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_taskflow PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_taskflow PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_taskflow ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_taskflow PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_taskflow
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_taskflow PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                           "$<INSTALL_INTERFACE:include>")

# Create target for Bullet implementation
add_library(${PROJECT_NAME}_taskflow_factories src/taskflow_task_composer_plugin_factories.cpp)
target_link_libraries(${PROJECT_NAME}_taskflow_factories PUBLIC ${PROJECT_NAME}_taskflow)
target_compile_options(${PROJECT_NAME}_taskflow_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_taskflow_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_taskflow_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_taskflow_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_taskflow_factories ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  ${PROJECT_NAME}_taskflow_factories
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_taskflow_factories PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                            "$<INSTALL_INTERFACE:include>")

# Add factory library so task_composer_factory can find these factories by defauult
set(TASK_COMPOSER_PLUGINS ${TASK_COMPOSER_PLUGINS} "${PROJECT_NAME}_taskflow_factories" PARENT_SCOPE)

# Mark header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/taskflow
  DESTINATION include/${PROJECT_NAME}
  COMPONENT taskflow
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

# Configure Components
configure_component(
  COMPONENT taskflow
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_taskflow ${PROJECT_NAME}_taskflow_factories
  DEPENDENCIES "tesseract_task_composer COMPONENTS core" Taskflow)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT taskflow
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract task composer taskflow components"
    COMPONENT_DEPENDS core
    LINUX_DEPENDS "taskflow | ${TESSERACT_PACKAGE_PREFIX}taskflow"
    WINDOWS_DEPENDS "taskflow | ${TESSERACT_PACKAGE_PREFIX}taskflow")
endif()
