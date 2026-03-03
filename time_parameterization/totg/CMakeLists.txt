add_library(time_parameterization_totg src/cereal_serialization.cpp src/time_optimal_trajectory_generation.cpp
                                       src/time_optimal_trajectory_generation_profiles.cpp)
add_library(tesseract::time_parameterization_totg ALIAS time_parameterization_totg)
set_target_properties(time_parameterization_totg PROPERTIES OUTPUT_NAME tesseract_time_parameterization_totg)
target_link_libraries(time_parameterization_totg PUBLIC tesseract::time_parameterization cereal::cereal)
target_include_directories(time_parameterization_totg PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                             "$<INSTALL_INTERFACE:include>")
target_compile_options(time_parameterization_totg PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(time_parameterization_totg PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(time_parameterization_totg PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(time_parameterization_totg ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(time_parameterization_totg PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  time_parameterization_totg
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

install(DIRECTORY include/tesseract DESTINATION include COMPONENT time_parameterization_totg)

if(TESSERACT_ENABLE_TESTING)
  add_subdirectory(test)
endif()

configure_component(
  COMPONENT time_parameterization_totg
  NAMESPACE tesseract
  TARGETS time_parameterization_totg
  DEPENDENCIES "tesseract_planning COMPONENTS time_parameterization" cereal)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT time_parameterization_totg
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract time parameterization TOTG components"
    COMPONENT_DEPENDS time_parameterization)
endif()
