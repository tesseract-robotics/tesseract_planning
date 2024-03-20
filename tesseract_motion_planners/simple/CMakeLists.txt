add_library(
  ${PROJECT_NAME}_simple
  src/interpolation.cpp
  src/simple_motion_planner.cpp
  src/profile/simple_planner_profile.cpp
  src/profile/simple_planner_lvs_plan_profile.cpp
  src/profile/simple_planner_lvs_no_ik_plan_profile.cpp
  src/profile/simple_planner_fixed_size_assign_plan_profile.cpp
  src/profile/simple_planner_fixed_size_plan_profile.cpp
  src/profile/simple_planner_lvs_assign_plan_profile.cpp
  src/profile/simple_planner_lvs_no_ik_plan_profile.cpp
  src/profile/simple_planner_lvs_plan_profile.cpp)
target_link_libraries(${PROJECT_NAME}_simple PUBLIC ${PROJECT_NAME}_core Boost::boost)
target_compile_options(${PROJECT_NAME}_simple PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_simple PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_simple PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_simple ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_simple PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_simple
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_simple PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:include>")
add_dependencies(${PROJECT_NAME}_simple ${PROJECT_NAME}_core)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include COMPONENT simple)

# Testing
if(TESSERACT_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# Configure Components
configure_component(
  COMPONENT simple
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_simple
  DEPENDENCIES "tesseract_motion_planners COMPONENTS core" tesseract_environment tesseract_command_language)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT simple
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract motion planners simple components"
    COMPONENT_DEPENDS core)
endif()
