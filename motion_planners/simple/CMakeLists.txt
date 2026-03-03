add_library(
  motion_planners_simple
  src/cereal_serialization.cpp
  src/interpolation.cpp
  src/simple_motion_planner.cpp
  src/profile/simple_planner_fixed_size_assign_no_ik_move_profile.cpp
  src/profile/simple_planner_fixed_size_assign_move_profile.cpp
  src/profile/simple_planner_fixed_size_move_profile.cpp
  src/profile/simple_planner_lvs_assign_no_ik_move_profile.cpp
  src/profile/simple_planner_lvs_assign_move_profile.cpp
  src/profile/simple_planner_lvs_no_ik_move_profile.cpp
  src/profile/simple_planner_lvs_move_profile.cpp
  src/profile/simple_planner_profile.cpp)
add_library(tesseract::motion_planners_simple ALIAS motion_planners_simple)
set_target_properties(motion_planners_simple PROPERTIES OUTPUT_NAME tesseract_motion_planners_simple)
target_link_libraries(motion_planners_simple PUBLIC tesseract::motion_planners Boost::boost cereal::cereal)
target_compile_options(motion_planners_simple PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(motion_planners_simple PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(motion_planners_simple PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(motion_planners_simple ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(motion_planners_simple PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  motion_planners_simple
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(motion_planners_simple PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:include>")
add_dependencies(motion_planners_simple motion_planners)

# Mark cpp header files for installation
install(DIRECTORY include/tesseract DESTINATION include COMPONENT motion_planners_simple)

# Testing
if(TESSERACT_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# Configure Components
configure_component(
  COMPONENT motion_planners_simple
  NAMESPACE tesseract
  TARGETS motion_planners_simple
  DEPENDENCIES "tesseract_planning COMPONENTS command_language motion_planners" "tesseract COMPONENTS environment"
               cereal)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT motion_planners_simple
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract motion planners simple components"
    COMPONENT_DEPENDS motion_planners)
endif()
