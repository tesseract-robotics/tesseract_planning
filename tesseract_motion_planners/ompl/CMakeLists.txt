find_package(ompl REQUIRED)

link_directories(BEFORE ${OMPL_LIBRARY_DIRS})

# OMPL Freespace Planning Interface
set(OMPL_SRC
    src/profile/ompl_profile.cpp
    src/profile/ompl_real_vector_plan_profile.cpp
    src/compound_state_validator.cpp
    src/continuous_motion_validator.cpp
    src/discrete_motion_validator.cpp
    src/ompl_motion_planner.cpp
    src/ompl_planner_configurator.cpp
    src/ompl_solver_config.cpp
    src/state_collision_validator.cpp
    src/utils.cpp
    src/weighted_real_vector_state_sampler.cpp)

# if(NOT OMPL_VERSION VERSION_LESS "1.4.0") list(APPEND OMPL_SRC src/config/ompl_planner_constrained_config.cpp) endif()

message(AUTHOR_WARNING "OMPL INCLUDE DIRS: ${OMPL_INCLUDE_DIRS}")
add_library(${PROJECT_NAME}_ompl ${OMPL_SRC})
target_link_libraries(
  ${PROJECT_NAME}_ompl
  PUBLIC ${PROJECT_NAME}_core
         ${PROJECT_NAME}_simple
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
  PRIVATE
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
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include COMPONENT ompl)

# Testing
if(TESSERACT_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# Configure Components
configure_component(
  COMPONENT ompl
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_ompl
  DEPENDENCIES "tesseract_motion_planners COMPONENTS core simple" ompl)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT ompl
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract motion planners ompl components"
    COMPONENT_DEPENDS core simple
    LINUX_DEPENDS "libompl-dev"
    WINDOWS_DEPENDS "ompl")
endif()
