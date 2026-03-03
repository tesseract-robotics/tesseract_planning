find_package(tesseract REQUIRED COMPONENTS kinematics_kdl)
add_library(time_parameterization_kdl src/cereal_serialization.cpp src/constant_tcp_speed_parameterization.cpp
                                      src/constant_tcp_speed_parameterization_profiles.cpp)
add_library(tesseract::time_parameterization_kdl ALIAS time_parameterization_kdl)
set_target_properties(time_parameterization_kdl PROPERTIES OUTPUT_NAME tesseract_time_parameterization_kdl)
target_link_libraries(time_parameterization_kdl PUBLIC tesseract::time_parameterization cereal::cereal)
target_link_libraries(time_parameterization_kdl PRIVATE tesseract::kinematics_kdl)
target_include_directories(time_parameterization_kdl PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                            "$<INSTALL_INTERFACE:include>")
target_compile_options(time_parameterization_kdl PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(time_parameterization_kdl PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(time_parameterization_kdl PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(time_parameterization_kdl ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(time_parameterization_kdl PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  time_parameterization_kdl
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

install(DIRECTORY include/tesseract DESTINATION include COMPONENT time_parameterization_kdl)

if(TESSERACT_ENABLE_TESTING)
  add_subdirectory(test)
endif()

configure_component(
  COMPONENT time_parameterization_kdl
  NAMESPACE tesseract
  TARGETS time_parameterization_kdl
  DEPENDENCIES "tesseract_planning COMPONENTS time_parameterization" cereal)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT time_parameterization_kdl
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract time parameterization KDL components"
    COMPONENT_DEPENDS time_parameterization)
endif()
