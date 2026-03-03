find_package(trajopt_ifopt REQUIRED)
find_package(trajopt_sqp REQUIRED)

# Trajopt IFOPT Planner
add_library(
  motion_planners_trajopt_ifopt
  src/cereal_serialization.cpp
  src/trajopt_ifopt_motion_planner.cpp
  src/trajopt_ifopt_utils.cpp
  src/trajopt_ifopt_waypoint_config.cpp
  src/profile/trajopt_ifopt_profile.cpp
  src/profile/trajopt_ifopt_default_move_profile.cpp
  src/profile/trajopt_ifopt_default_composite_profile.cpp
  src/profile/trajopt_ifopt_osqp_solver_profile.cpp)
add_library(tesseract::motion_planners_trajopt_ifopt ALIAS motion_planners_trajopt_ifopt)
set_target_properties(motion_planners_trajopt_ifopt PROPERTIES OUTPUT_NAME tesseract_motion_planners_trajopt_ifopt)
target_link_libraries(
  motion_planners_trajopt_ifopt
  PUBLIC tesseract::motion_planners
         tesseract::motion_planners_simple
         trajopt::trajopt_ifopt
         trajopt::trajopt_sqp
         Boost::boost
         cereal::cereal
         Eigen3::Eigen)
target_compile_options(motion_planners_trajopt_ifopt PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(motion_planners_trajopt_ifopt PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(motion_planners_trajopt_ifopt PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(motion_planners_trajopt_ifopt ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(motion_planners_trajopt_ifopt PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  motion_planners_trajopt_ifopt
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(motion_planners_trajopt_ifopt PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                                "$<INSTALL_INTERFACE:include>")

# Mark cpp header files for installation
install(DIRECTORY include/tesseract DESTINATION include COMPONENT motion_planners_trajopt_ifopt)

# Testing
if(TESSERACT_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# Configure Components
configure_component(
  COMPONENT motion_planners_trajopt_ifopt
  NAMESPACE tesseract
  TARGETS motion_planners_trajopt_ifopt
  DEPENDENCIES
    "tesseract_planning COMPONENTS motion_planners motion_planners_simple"
    trajopt_ifopt
    trajopt_sqp
    cereal)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT motion_planners_trajopt_ifopt
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract motion planners trajopt_ifopt components"
    COMPONENT_DEPENDS motion_planners motion_planners_simple
    LINUX_DEPENDS ${TESSERACT_PACKAGE_PREFIX}trajopt-ifopt ${TESSERACT_PACKAGE_PREFIX}trajopt-sqp
    WINDOWS_DEPENDS ${TESSERACT_PACKAGE_PREFIX}trajopt-ifopt ${TESSERACT_PACKAGE_PREFIX}trajopt-sqp)
endif()
