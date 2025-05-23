find_package(tesseract_kinematics REQUIRED COMPONENTS core kdl)
add_library(${PROJECT_NAME}_kdl src/constant_tcp_speed_parameterization.cpp
                                src/constant_tcp_speed_parameterization_profiles.cpp)
target_link_libraries(${PROJECT_NAME}_kdl PUBLIC ${PROJECT_NAME}_core)
target_link_libraries(${PROJECT_NAME}_kdl PRIVATE tesseract::tesseract_kinematics_kdl)
target_include_directories(${PROJECT_NAME}_kdl PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                      "$<INSTALL_INTERFACE:include>")
target_compile_options(${PROJECT_NAME}_kdl PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_kdl PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_kdl PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_kdl ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_kdl PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_kdl
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include COMPONENT kdl)

# Testing
if(TESSERACT_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# Configure Components
configure_component(
  COMPONENT kdl
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_kdl
  DEPENDENCIES "tesseract_time_parameterization COMPONENTS core")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT kdl
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract time parameterization KDL components"
    COMPONENT_DEPENDS core)
endif()
