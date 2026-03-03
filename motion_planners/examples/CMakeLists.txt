find_package(tesseract REQUIRED COMPONENTS visualization)

if(TESSERACT_BUILD_SIMPLE AND TESSERACT_BUILD_DESCARTES AND TESSERACT_BUILD_TRAJOPT)
  message("Building chain example")
  add_executable(motion_planners_chain_example chain_example.cpp)
  target_link_libraries(
    motion_planners_chain_example
    motion_planners_descartes
    motion_planners_trajopt
    motion_planners_simple
    tesseract::state_solver_ofkt
    tesseract::visualization)
  target_compile_options(motion_planners_chain_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                               ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(motion_planners_chain_example PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(motion_planners_chain_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(motion_planners_chain_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  install(TARGETS motion_planners_chain_example DESTINATION bin)
endif()

if(TESSERACT_BUILD_SIMPLE AND TESSERACT_BUILD_DESCARTES AND TESSERACT_BUILD_TRAJOPT)
  message("Building raster example")
  add_executable(motion_planners_raster_example raster_example.cpp)
  target_link_libraries(
    motion_planners_raster_example
    motion_planners_descartes
    motion_planners_trajopt
    motion_planners_simple
    tesseract::state_solver_ofkt
    tesseract::visualization)
  target_compile_options(motion_planners_raster_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(motion_planners_raster_example PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(motion_planners_raster_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(motion_planners_raster_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  install(TARGETS motion_planners_raster_example DESTINATION bin)
endif()

if(TESSERACT_BUILD_SIMPLE AND TESSERACT_BUILD_OMPL AND TESSERACT_BUILD_TRAJOPT)
  message("Building freespace example")
  add_executable(motion_planners_freespace_example freespace_example.cpp)
  target_link_libraries(
    motion_planners_freespace_example
    motion_planners_ompl
    motion_planners_trajopt
    motion_planners_simple
    tesseract::state_solver_ofkt
    tesseract::visualization)
  target_compile_options(motion_planners_freespace_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                   ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(motion_planners_freespace_example PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_clang_tidy(motion_planners_freespace_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(motion_planners_freespace_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  install(TARGETS motion_planners_freespace_example DESTINATION bin)
endif()
