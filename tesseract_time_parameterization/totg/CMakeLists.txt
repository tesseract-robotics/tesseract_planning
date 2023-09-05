add_library(${PROJECT_NAME}_totg src/time_optimal_trajectory_generation.cpp)
target_link_libraries(${PROJECT_NAME}_totg PUBLIC ${PROJECT_NAME}_core)
target_include_directories(${PROJECT_NAME}_totg PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                       "$<INSTALL_INTERFACE:include>")
target_compile_options(${PROJECT_NAME}_totg PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_totg PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_totg PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_totg ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_totg PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_totg
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include COMPONENT totg)

# Testing
if(TESSERACT_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# Configure Components
configure_component(
  COMPONENT totg
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_totg
  DEPENDENCIES "tesseract_time_parameterization COMPONENTS core")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT totg
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract time parameterization TOTG components"
    COMPONENT_DEPENDS core)
endif()
