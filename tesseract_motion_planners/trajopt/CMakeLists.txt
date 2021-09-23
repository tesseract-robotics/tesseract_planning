find_package(trajopt REQUIRED)
find_package(trajopt_sco REQUIRED)

# Trajopt Planner
add_library(
  ${PROJECT_NAME}_trajopt
  src/trajopt_collision_config.cpp
  src/trajopt_motion_planner.cpp
  src/trajopt_utils.cpp
  src/profile/trajopt_default_plan_profile.cpp
  src/profile/trajopt_default_composite_profile.cpp
  src/profile/trajopt_default_solver_profile.cpp
  src/problem_generators/default_problem_generator.cpp
  src/serialize.cpp
  src/deserialize.cpp)
target_link_libraries(
  ${PROJECT_NAME}_trajopt
  PUBLIC Boost::boost
         Eigen3::Eigen
         ${PROJECT_NAME}_core
         trajopt::trajopt
         trajopt::trajopt_utils
         trajopt::trajopt_sco)
target_compile_options(${PROJECT_NAME}_trajopt PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_trajopt PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_trajopt PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_trajopt ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_trajopt PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_trajopt
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_trajopt PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                          "$<INSTALL_INTERFACE:include>")

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Install targets
install_targets(TARGETS ${PROJECT_NAME}_trajopt)
