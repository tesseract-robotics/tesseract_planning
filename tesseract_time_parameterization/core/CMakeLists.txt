target_sources(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/instructions_trajectory.cpp>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/instructions_trajectory.cpp>")
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC tesseract::tesseract_common
         tesseract::tesseract_command_language
         console_bridge::console_bridge
         Eigen3::Eigen)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")

# Mark header files for installation
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
