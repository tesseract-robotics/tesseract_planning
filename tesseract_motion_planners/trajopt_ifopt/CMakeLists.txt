find_package(trajopt_ifopt REQUIRED)
find_package(trajopt_sqp REQUIRED)

# Trajopt IFOPT Planner
add_library(
  ${PROJECT_NAME}_trajopt_ifopt SHARED
  src/trajopt_ifopt_motion_planner.cpp
  src/trajopt_ifopt_utils.cpp
  src/profile/trajopt_ifopt_default_plan_profile.cpp
  src/profile/trajopt_ifopt_default_composite_profile.cpp
  src/profile/trajopt_ifopt_default_solver_profile.cpp)
target_link_libraries(
  ${PROJECT_NAME}_trajopt_ifopt
  PUBLIC ${PROJECT_NAME}_core
         ${PROJECT_NAME}_simple
         trajopt::trajopt_ifopt
         trajopt::trajopt_sqp
         Boost::boost
         Eigen3::Eigen)
target_compile_options(${PROJECT_NAME}_trajopt_ifopt PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_trajopt_ifopt PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_trajopt_ifopt PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_trajopt_ifopt ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_trajopt_ifopt PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_trajopt_ifopt
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_trajopt_ifopt PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                                "$<INSTALL_INTERFACE:include>")

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include COMPONENT trajopt_ifopt)

# Configure Components
configure_component(
  COMPONENT trajopt_ifopt
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_trajopt_ifopt
  DEPENDENCIES "tesseract_motion_planners COMPONENTS core simple" trajopt_ifopt trajopt_sqp)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT trajopt_ifopt
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract motion planners trajopt_ifopt components"
    COMPONENT_DEPENDS core simple
    LINUX_DEPENDS ${TESSERACT_PACKAGE_PREFIX}trajopt-ifopt ${TESSERACT_PACKAGE_PREFIX}trajopt-sqp
    WINDOWS_DEPENDS ${TESSERACT_PACKAGE_PREFIX}trajopt-ifopt ${TESSERACT_PACKAGE_PREFIX}trajopt-sqp)
endif()
