find_package(tesseract_environment REQUIRED)

add_executable(${PROJECT_NAME}_freespace_manager_example freespace_manager_example.cpp)
target_link_libraries(
  ${PROJECT_NAME}_freespace_manager_example
  console_bridge::console_bridge
  Eigen3::Eigen
  ${PROJECT_NAME}
  ${PROJECT_NAME}_utils
  tesseract::tesseract_environment
  tesseract::tesseract_command_language
  tesseract::tesseract_support
  ${CMAKE_THREAD_LIBS_INIT})
target_compile_options(${PROJECT_NAME}_freespace_manager_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                         ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_freespace_manager_example PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_freespace_manager_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_freespace_manager_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(${PROJECT_NAME}_freespace_manager_example
                           PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
list(APPEND Examples ${PROJECT_NAME}_freespace_manager_example)

add_executable(${PROJECT_NAME}_raster_manager_example raster_manager_example.cpp)
target_link_libraries(
  ${PROJECT_NAME}_raster_manager_example
  console_bridge::console_bridge
  Eigen3::Eigen
  ${PROJECT_NAME}
  ${PROJECT_NAME}_utils
  tesseract::tesseract_environment
  tesseract::tesseract_command_language
  tesseract::tesseract_support
  ${CMAKE_THREAD_LIBS_INIT})
target_compile_options(${PROJECT_NAME}_raster_manager_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                      ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_raster_manager_example PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_raster_manager_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_raster_manager_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(${PROJECT_NAME}_raster_manager_example
                           PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
list(APPEND Examples ${PROJECT_NAME}_raster_manager_example)

if(NOT WIN32)
  add_executable(${PROJECT_NAME}_memory_usage_example memory_usage_example.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_memory_usage_example
    console_bridge::console_bridge
    Eigen3::Eigen
    ${PROJECT_NAME}
    tesseract::tesseract_environment
    tesseract::tesseract_command_language
    tesseract::tesseract_support
    ${CMAKE_THREAD_LIBS_INIT})
  target_compile_options(${PROJECT_NAME}_memory_usage_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                      ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${PROJECT_NAME}_memory_usage_example PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(${PROJECT_NAME}_memory_usage_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${PROJECT_NAME}_memory_usage_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_include_directories(${PROJECT_NAME}_memory_usage_example
                             PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
  list(APPEND Examples ${PROJECT_NAME}_memory_usage_example)
endif()

add_executable(${PROJECT_NAME}_taskflow_profiling_example taskflow_profiling_example.cpp)
target_link_libraries(
  ${PROJECT_NAME}_taskflow_profiling_example
  console_bridge::console_bridge
  Eigen3::Eigen
  ${PROJECT_NAME}
  ${PROJECT_NAME}_utils
  tesseract::tesseract_environment
  tesseract::tesseract_command_language
  tesseract::tesseract_support
  ${CMAKE_THREAD_LIBS_INIT})
target_compile_options(${PROJECT_NAME}_taskflow_profiling_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                          ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_taskflow_profiling_example PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_taskflow_profiling_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_taskflow_profiling_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(${PROJECT_NAME}_taskflow_profiling_example
                           PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
list(APPEND Examples ${PROJECT_NAME}_taskflow_profiling_example)

install(
  TARGETS ${Examples}
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
