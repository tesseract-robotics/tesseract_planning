find_package(tesseract_environment REQUIRED)
find_package(tesseract_command_language REQUIRED)

# Create interface for core
add_library(${PROJECT_NAME}_core src/core/utils.cpp)
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  tesseract::tesseract_environment_core
  tesseract::tesseract_common
  tesseract::tesseract_command_language
  console_bridge::console_bridge
  Eigen3::Eigen)
target_compile_options(${PROJECT_NAME}_core PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_core PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_core PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_core ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_core PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME}_core ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_core PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Install targets
install_targets(TARGETS ${PROJECT_NAME}_core)
