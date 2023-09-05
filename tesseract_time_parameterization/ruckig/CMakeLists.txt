find_package(ruckig REQUIRED)

add_library(${PROJECT_NAME}_ruckig src/ruckig_trajectory_smoothing.cpp)
target_link_libraries(${PROJECT_NAME}_ruckig PUBLIC ${PROJECT_NAME}_core ruckig::ruckig)
target_include_directories(${PROJECT_NAME}_ruckig PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:include>")
target_compile_options(${PROJECT_NAME}_ruckig PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_ruckig PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_ruckig PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_ruckig ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_ruckig PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_ruckig
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include COMPONENT ruckig)

# Testing
if(TESSERACT_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# Configure Components
configure_component(
  COMPONENT ruckig
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_ruckig
  DEPENDENCIES "tesseract_time_parameterization COMPONENTS core" ruckig)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT ruckig
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract time parameterization ruckig components"
    COMPONENT_DEPENDS core
    LINUX_DEPENDS "ruckig | ${TESSERACT_PACKAGE_PREFIX}ruckig"
    WINDOWS_DEPENDS "ruckig | ${TESSERACT_PACKAGE_PREFIX}ruckig")
endif()
