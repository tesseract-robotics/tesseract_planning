find_package(trajopt REQUIRED)
find_package(trajopt_sco REQUIRED)
find_package(osqp REQUIRED)

# Trajopt Planner
add_library(
  motion_planners_trajopt
  src/cereal_serialization.cpp
  src/trajopt_motion_planner.cpp
  src/trajopt_utils.cpp
  src/trajopt_waypoint_config.cpp
  src/profile/trajopt_profile.cpp
  src/profile/trajopt_default_move_profile.cpp
  src/profile/trajopt_default_composite_profile.cpp
  src/profile/trajopt_osqp_solver_profile.cpp)
add_library(tesseract::motion_planners_trajopt ALIAS motion_planners_trajopt)
set_target_properties(motion_planners_trajopt PROPERTIES OUTPUT_NAME tesseract_motion_planners_trajopt)
target_link_libraries(
  motion_planners_trajopt
  PUBLIC tesseract::motion_planners
         tesseract::motion_planners_simple
         trajopt::trajopt
         trajopt::trajopt_common
         trajopt::trajopt_sco
         tesseract::kinematics
         tesseract::environment
         Boost::boost
         cereal::cereal
         Eigen3::Eigen
         osqp::osqp)
target_compile_options(motion_planners_trajopt PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(motion_planners_trajopt PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(motion_planners_trajopt PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(motion_planners_trajopt ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(motion_planners_trajopt PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  motion_planners_trajopt
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(motion_planners_trajopt PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                          "$<INSTALL_INTERFACE:include>")

# Mark cpp header files for installation
install(DIRECTORY include/tesseract DESTINATION include COMPONENT motion_planners_trajopt)

# Testing
if(TESSERACT_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# Configure Components
configure_component(
  COMPONENT motion_planners_trajopt
  NAMESPACE tesseract
  TARGETS motion_planners_trajopt
  DEPENDENCIES
    "tesseract_planning COMPONENTS motion_planners motion_planners_simple"
    trajopt
    trajopt_sco
    osqp
    cereal)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT motion_planners_trajopt
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract motion planners trajopt components"
    COMPONENT_DEPENDS motion_planners motion_planners_simple
    LINUX_DEPENDS ${TESSERACT_PACKAGE_PREFIX}trajopt ${TESSERACT_PACKAGE_PREFIX}trajopt-sco
    WINDOWS_DEPENDS ${TESSERACT_PACKAGE_PREFIX}trajopt ${TESSERACT_PACKAGE_PREFIX}trajopt-sco)
endif()
