find_package(tesseract_support REQUIRED)
find_package(tesseract_visualization REQUIRED)

if(TESSERACT_BUILD_DESCARTES AND TESSERACT_BUILD_TRAJOPT)
  message("Building chain example")
  add_executable(${PROJECT_NAME}_chain_example chain_example.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_chain_example
    ${PROJECT_NAME}_descartes
    ${PROJECT_NAME}_trajopt
    ${PROJECT_NAME}_simple
    tesseract::tesseract_state_solver_ofkt
    tesseract::tesseract_support
    tesseract::tesseract_visualization)
  target_compile_options(${PROJECT_NAME}_chain_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                               ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${PROJECT_NAME}_chain_example PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(${PROJECT_NAME}_chain_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${PROJECT_NAME}_chain_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  install(TARGETS ${PROJECT_NAME}_chain_example DESTINATION bin)
endif()

if(TESSERACT_BUILD_DESCARTES AND TESSERACT_BUILD_TRAJOPT)
  message("Building raster example")
  add_executable(${PROJECT_NAME}_raster_example raster_example.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_raster_example
    ${PROJECT_NAME}_descartes
    ${PROJECT_NAME}_trajopt
    ${PROJECT_NAME}_simple
    tesseract::tesseract_state_solver_ofkt
    tesseract::tesseract_support
    tesseract::tesseract_visualization)
  target_compile_options(${PROJECT_NAME}_raster_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${PROJECT_NAME}_raster_example PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(${PROJECT_NAME}_raster_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${PROJECT_NAME}_raster_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  install(TARGETS ${PROJECT_NAME}_raster_example DESTINATION bin)
endif()

if(TESSERACT_BUILD_OMPL AND TESSERACT_BUILD_TRAJOPT)
  message("Building freespace example")
  add_executable(${PROJECT_NAME}_freespace_example freespace_example.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_freespace_example
    ${PROJECT_NAME}_ompl
    ${PROJECT_NAME}_trajopt
    ${PROJECT_NAME}_simple
    tesseract::tesseract_state_solver_ofkt
    tesseract::tesseract_support
    tesseract::tesseract_visualization)
  target_compile_options(${PROJECT_NAME}_freespace_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                   ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${PROJECT_NAME}_freespace_example PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(${PROJECT_NAME}_freespace_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${PROJECT_NAME}_freespace_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  install(TARGETS ${PROJECT_NAME}_freespace_example DESTINATION bin)
endif()
