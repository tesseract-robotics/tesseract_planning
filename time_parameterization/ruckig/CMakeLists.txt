find_package(ruckig REQUIRED)

add_library(time_parameterization_ruckig src/cereal_serialization.cpp src/ruckig_trajectory_smoothing.cpp
                                         src/ruckig_trajectory_smoothing_profiles.cpp)
add_library(tesseract::time_parameterization_ruckig ALIAS time_parameterization_ruckig)
set_target_properties(time_parameterization_ruckig PROPERTIES OUTPUT_NAME tesseract_time_parameterization_ruckig)
target_link_libraries(time_parameterization_ruckig PUBLIC tesseract::time_parameterization ruckig::ruckig
                                                          cereal::cereal)
target_include_directories(time_parameterization_ruckig PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                               "$<INSTALL_INTERFACE:include>")
target_compile_options(time_parameterization_ruckig PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(time_parameterization_ruckig PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(time_parameterization_ruckig PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(time_parameterization_ruckig ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(time_parameterization_ruckig PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  time_parameterization_ruckig
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

install(DIRECTORY include/tesseract DESTINATION include COMPONENT time_parameterization_ruckig)

if(TESSERACT_ENABLE_TESTING)
  add_subdirectory(test)
endif()

configure_component(
  COMPONENT time_parameterization_ruckig
  NAMESPACE tesseract
  TARGETS time_parameterization_ruckig
  DEPENDENCIES "tesseract_planning COMPONENTS time_parameterization" ruckig cereal)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT time_parameterization_ruckig
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract time parameterization ruckig components"
    COMPONENT_DEPENDS time_parameterization
    LINUX_DEPENDS "ruckig | ${TESSERACT_PACKAGE_PREFIX}ruckig"
    WINDOWS_DEPENDS "ruckig | ${TESSERACT_PACKAGE_PREFIX}ruckig")
endif()
