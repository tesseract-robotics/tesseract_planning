find_package(tesseract_environment REQUIRED)
find_package(tesseract_visualization REQUIRED)
find_package(tesseract_support REQUIRED)
find_package(Taskflow REQUIRED)

add_executable(${PROJECT_NAME}_example task_composer_example.cpp)
target_link_libraries(
  ${PROJECT_NAME}_example
  PRIVATE ${PROJECT_NAME}
          ${PROJECT_NAME}_taskflow
          console_bridge::console_bridge
          tesseract::tesseract_environment
          tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                       ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_example PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(${PROJECT_NAME}_example PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
list(APPEND Examples ${PROJECT_NAME}_example)

add_executable(${PROJECT_NAME}_trajopt_example task_composer_trajopt_example.cpp)
target_link_libraries(
  ${PROJECT_NAME}_trajopt_example
  PRIVATE ${PROJECT_NAME}
          ${PROJECT_NAME}_planning
          ${PROJECT_NAME}_taskflow
          console_bridge::console_bridge
          tesseract::tesseract_environment
          tesseract::tesseract_visualization
          tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_trajopt_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                               ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_trajopt_example PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_trajopt_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_trajopt_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(${PROJECT_NAME}_trajopt_example PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
list(APPEND Examples ${PROJECT_NAME}_trajopt_example)

add_executable(${PROJECT_NAME}_raster_example task_composer_raster_example.cpp)
target_link_libraries(
  ${PROJECT_NAME}_raster_example
  PRIVATE ${PROJECT_NAME}
          ${PROJECT_NAME}_planning
          ${PROJECT_NAME}_taskflow
          console_bridge::console_bridge
          tesseract::tesseract_environment
          tesseract::tesseract_visualization
          tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_raster_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                              ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_raster_example PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_raster_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_raster_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(${PROJECT_NAME}_raster_example PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
list(APPEND Examples ${PROJECT_NAME}_raster_example)

install(
  TARGETS ${Examples}
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
