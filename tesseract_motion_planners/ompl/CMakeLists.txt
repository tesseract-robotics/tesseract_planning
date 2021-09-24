find_package(ompl REQUIRED)

link_directories(BEFORE ${OMPL_LIBRARY_DIRS})

# OMPL Freespace Planning Interface
set(OMPL_SRC
    src/ompl_motion_planner.cpp
    src/continuous_motion_validator.cpp
    src/discrete_motion_validator.cpp
    src/weighted_real_vector_state_sampler.cpp
    src/ompl_motion_planner_status_category.cpp
    src/ompl_planner_configurator.cpp
    src/ompl_problem.cpp
    src/profile/ompl_default_plan_profile.cpp
    src/problem_generators/default_problem_generator.cpp
    src/utils.cpp
    src/state_collision_validator.cpp
    src/compound_state_validator.cpp
    src/serialize.cpp
    src/deserialize.cpp)

# if(NOT OMPL_VERSION VERSION_LESS "1.4.0") list(APPEND OMPL_SRC src/config/ompl_planner_constrained_config.cpp) endif()

message(AUTHOR_WARNING "OMPL INCLUDE DIRS: ${OMPL_INCLUDE_DIRS}")
add_library(${PROJECT_NAME}_ompl ${OMPL_SRC})
target_link_libraries(
  ${PROJECT_NAME}_ompl
  PUBLIC ${PROJECT_NAME}_core
         ${OMPL_LIBRARIES}
         Boost::boost
         ${CMAKE_THREAD_LIBS_INIT})
target_compile_options(${PROJECT_NAME}_ompl PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_ompl PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_ompl PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_ompl ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_ompl PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_ompl
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_ompl PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                       "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_ompl SYSTEM PUBLIC ${OMPL_INCLUDE_DIRS})
if(OMPL_VERSION VERSION_LESS "1.4.0")
  target_compile_definitions(${PROJECT_NAME}_ompl PUBLIC OMPL_LESS_1_4_0=ON)
endif()

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Install targets
install_targets(TARGETS ${PROJECT_NAME}_ompl)
