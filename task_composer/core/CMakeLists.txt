# Need to create export header so static class variables can be exported on windows build
include(GenerateExportHeader)

find_package(Boost REQUIRED COMPONENTS program_options)

add_library(
  task_composer
  src/cereal_serialization.cpp
  src/task_composer_data_storage.cpp
  src/task_composer_context.cpp
  src/task_composer_executor.cpp
  src/task_composer_future.cpp
  src/task_composer_graph.cpp
  src/task_composer_keys.cpp
  src/task_composer_log.cpp
  src/task_composer_node_info.cpp
  src/task_composer_node_ports.cpp
  src/task_composer_node.cpp
  src/task_composer_pipeline.cpp
  src/task_composer_plugin_factory.cpp
  src/task_composer_server.cpp
  src/task_composer_task.cpp
  src/yaml_utils.cpp)
add_library(tesseract::task_composer ALIAS task_composer)
set_target_properties(task_composer PROPERTIES OUTPUT_NAME tesseract_task_composer)

target_link_libraries(
  task_composer
  PUBLIC console_bridge::console_bridge
         tesseract::common
         boost_plugin_loader::boost_plugin_loader
         Boost::boost
         cereal::cereal
         yaml-cpp)
target_compile_options(task_composer PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(task_composer PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(
  task_composer PUBLIC ${TESSERACT_COMPILE_DEFINITIONS}
                       TESSERACT_TASK_COMPOSER_DIR="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/task_composer")
target_compile_definitions(task_composer
                           PRIVATE TESSERACT_TASK_COMPOSER_PLUGIN_PATH="${TESSERACT_TASK_COMPOSER_PLUGIN_PATH}")
target_clang_tidy(task_composer ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(task_composer PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  task_composer
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  task_composer PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                       "$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/../>" "$<INSTALL_INTERFACE:include>")

add_library(
  task_composer_nodes
  src/nodes/done_task.cpp
  src/nodes/error_task.cpp
  src/nodes/for_each_task.cpp
  src/nodes/has_data_storage_entry_task.cpp
  src/nodes/remap_task.cpp
  src/nodes/start_task.cpp
  src/nodes/sync_task.cpp
  src/test_suite/test_task.cpp)
add_library(tesseract::task_composer_nodes ALIAS task_composer_nodes)
set_target_properties(task_composer_nodes PROPERTIES OUTPUT_NAME tesseract_task_composer_nodes)

generate_export_header(
  task_composer_nodes
  BASE_NAME
  tesseract_task_composer_nodes
  EXPORT_FILE_NAME
  ${PROJECT_BINARY_DIR}/tesseract/task_composer/tesseract_task_composer_nodes_export.h)

target_link_libraries(
  task_composer_nodes
  PUBLIC tesseract::task_composer
         console_bridge::console_bridge
         tesseract::common
         Boost::boost
         cereal::cereal
         yaml-cpp)
target_compile_options(task_composer_nodes PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(task_composer_nodes PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(task_composer_nodes PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(task_composer_nodes ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(task_composer_nodes PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  task_composer_nodes
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  task_composer_nodes PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                             "$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>" "$<INSTALL_INTERFACE:include>")

# Create target for factory implementation
add_library(task_composer_factories src/task_composer_task_plugin_factory.cpp)
set_target_properties(task_composer_factories PROPERTIES OUTPUT_NAME tesseract_task_composer_factories)
target_link_libraries(task_composer_factories PUBLIC tesseract::task_composer tesseract::task_composer_nodes)
target_compile_options(task_composer_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(task_composer_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(task_composer_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(task_composer_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(task_composer_factories ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  task_composer_factories
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(task_composer_factories PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                          "$<INSTALL_INTERFACE:include>")

# Add factory library so task_composer_factory can find these factories by default
set(TASK_COMPOSER_PLUGINS ${TASK_COMPOSER_PLUGINS} "tesseract_task_composer_factories" PARENT_SCOPE)

# Tools
add_executable(task_composer_validate_config src/tools/task_composer_validate_config.cpp)
set_target_properties(task_composer_validate_config PROPERTIES OUTPUT_NAME tesseract_task_composer_validate_config)
target_link_libraries(task_composer_validate_config PRIVATE tesseract::task_composer Boost::program_options)
target_compile_options(task_composer_validate_config PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(task_composer_validate_config PRIVATE ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(task_composer_validate_config PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(task_composer_validate_config PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(task_composer_validate_config ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_include_directories(
  task_composer_validate_config PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                        "$<INSTALL_INTERFACE:include>")
install(
  DIRECTORY include/tesseract/task_composer
  DESTINATION include/tesseract
  COMPONENT task_composer
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")

install(FILES ${PROJECT_BINARY_DIR}/tesseract/task_composer/tesseract_task_composer_nodes_export.h
        DESTINATION include/tesseract/task_composer/ COMPONENT task_composer)

# Plugin should not be export only installed
install(
  TARGETS task_composer_factories
  COMPONENT task_composer
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# Configure Components
configure_component(
  COMPONENT task_composer
  NAMESPACE tesseract
  TARGETS task_composer task_composer_nodes task_composer_validate_config
  DEPENDENCIES
    console_bridge
    "tesseract COMPONENTS common"
    boost_plugin_loader
    cereal
    yaml-cpp)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT task_composer
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract task composer core components"
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libyaml-cpp-dev"
      "boost-plugin-loader"
      "Boost REQUIRED COMPONENTS program_options"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
    WINDOWS_DEPENDS "console-bridge" "yaml-cpp" "${TESSERACT_PACKAGE_PREFIX}tesseract-common")
endif()
