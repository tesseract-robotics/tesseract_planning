add_library(${PROJECT_NAME}_isp src/iterative_spline_parameterization.cpp
                                src/iterative_spline_parameterization_profiles.cpp)
target_link_libraries(${PROJECT_NAME}_isp PUBLIC ${PROJECT_NAME}_core cereal::cereal)
target_include_directories(${PROJECT_NAME}_isp PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                      "$<INSTALL_INTERFACE:include>")
target_compile_options(${PROJECT_NAME}_isp PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_isp PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_isp PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_isp ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_isp PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_isp
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include COMPONENT isp)

# Testing
if(TESSERACT_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# Configure Components
configure_component(
  COMPONENT isp
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_isp
  DEPENDENCIES "tesseract_time_parameterization COMPONENTS core" cereal)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT isp
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract time parameterization ISP components"
    COMPONENT_DEPENDS core)
endif()
