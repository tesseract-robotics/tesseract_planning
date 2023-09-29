add_library(
  ${PROJECT_NAME}
  src/task_composer_data_storage.cpp
  src/task_composer_context.cpp
  src/task_composer_executor.cpp
  src/task_composer_graph.cpp
  src/task_composer_node.cpp
  src/task_composer_node_info.cpp
  src/task_composer_pipeline.cpp
  src/task_composer_plugin_factory.cpp
  src/task_composer_problem.cpp
  src/task_composer_server.cpp
  src/task_composer_task.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC console_bridge::console_bridge
         tesseract::tesseract_common
         Boost::boost
         Boost::serialization
         yaml-cpp)
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(
  ${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_DEFINITIONS}
                         TESSERACT_TASK_COMPOSER_DIR="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")
target_compile_definitions(${PROJECT_NAME}
                           PRIVATE TESSERACT_TASK_COMPOSER_PLUGIN_PATH="${TESSERACT_TASK_COMPOSER_PLUGIN_PATH}")
target_clang_tidy(${PROJECT_NAME} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")

add_library(
  ${PROJECT_NAME}_nodes
  src/nodes/done_task.cpp
  src/nodes/error_task.cpp
  src/nodes/remap_task.cpp
  src/nodes/start_task.cpp
  src/nodes/sync_task.cpp
  src/test_suite/test_task.cpp)
target_link_libraries(
  ${PROJECT_NAME}_nodes
  PUBLIC ${PROJECT_NAME}
         console_bridge::console_bridge
         tesseract::tesseract_common
         Boost::boost
         Boost::serialization
         yaml-cpp)
target_compile_options(${PROJECT_NAME}_nodes PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_nodes PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_nodes PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_nodes ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_nodes PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_nodes
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_nodes PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                        "$<INSTALL_INTERFACE:include>")

# Create target for Bullet implementation
add_library(${PROJECT_NAME}_factories src/task_composer_task_plugin_factory.cpp)
target_link_libraries(${PROJECT_NAME}_factories PUBLIC ${PROJECT_NAME} ${PROJECT_NAME}_nodes)
target_compile_options(${PROJECT_NAME}_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_factories ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  ${PROJECT_NAME}_factories
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_factories PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                            "$<INSTALL_INTERFACE:include>")

# Add factory library so task_composer_factory can find these factories by defauult
set(TASK_COMPOSER_PLUGINS ${TASK_COMPOSER_PLUGINS} "${PROJECT_NAME}_factories" PARENT_SCOPE)

install(
  DIRECTORY include/${PROJECT_NAME}/core
  DESTINATION include/${PROJECT_NAME}
  COMPONENT core
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")

# Configure Components
configure_component(
  COMPONENT core
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_nodes ${PROJECT_NAME}_factories
  DEPENDENCIES
    console_bridge
    tesseract_common
    "Boost REQUIRED COMPONENTS serialization"
    yaml-cpp
  CFG_EXTRAS cmake/core-extras.cmake)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT core
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract task composer core components"
    LINUX_DEPENDS "libconsole-bridge-dev" "libyaml-cpp-dev" "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
    WINDOWS_DEPENDS "console-bridge" "yaml-cpp" "${TESSERACT_PACKAGE_PREFIX}tesseract-common")
endif()
