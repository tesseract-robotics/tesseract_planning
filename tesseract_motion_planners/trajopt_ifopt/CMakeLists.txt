find_package(trajopt_ifopt REQUIRED)
find_package(trajopt_sqp REQUIRED)

# Trajopt IFOPT Planner
add_library(
  ${PROJECT_NAME}_trajopt_ifopt SHARED
  src/trajopt_ifopt_motion_planner.cpp
  src/trajopt_ifopt_utils.cpp
  src/profile/trajopt_ifopt_default_plan_profile.cpp
  src/profile/trajopt_ifopt_default_composite_profile.cpp
  src/problem_generators/default_problem_generator.cpp)
target_link_libraries(
  ${PROJECT_NAME}_trajopt_ifopt
  PUBLIC Boost::boost
         Eigen3::Eigen
         ${PROJECT_NAME}_core
         trajopt::trajopt_ifopt
         trajopt::trajopt_sqp)
target_compile_options(${PROJECT_NAME}_trajopt_ifopt PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_trajopt_ifopt PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_trajopt_ifopt PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_trajopt_ifopt ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_trajopt_ifopt PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_trajopt_ifopt
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_TESTING})
target_include_directories(${PROJECT_NAME}_trajopt_ifopt PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                                "$<INSTALL_INTERFACE:include>")

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Install targets
install_targets(TARGETS ${PROJECT_NAME}_trajopt_ifopt)
