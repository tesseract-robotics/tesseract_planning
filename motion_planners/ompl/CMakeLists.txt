find_package(ompl REQUIRED)

link_directories(BEFORE ${OMPL_LIBRARY_DIRS})

# OMPL Freespace Planning Interface
set(OMPL_SRC
    src/profile/ompl_profile.cpp
    src/profile/ompl_real_vector_move_profile.cpp
    src/cereal_serialization.cpp
    src/collision_cost_objective.cpp
    src/compound_state_validator.cpp
    src/continuous_motion_validator.cpp
    src/discrete_motion_validator.cpp
    src/ompl_motion_planner.cpp
    src/ompl_planner_configurator.cpp
    src/ompl_solver_config.cpp
    src/state_collision_validator.cpp
    src/utils.cpp
    src/weighted_real_vector_state_sampler.cpp)

message(AUTHOR_WARNING "OMPL INCLUDE DIRS: ${OMPL_INCLUDE_DIRS}")
add_library(motion_planners_ompl ${OMPL_SRC})
add_library(tesseract::motion_planners_ompl ALIAS motion_planners_ompl)
set_target_properties(motion_planners_ompl PROPERTIES OUTPUT_NAME tesseract_motion_planners_ompl)
target_link_libraries(
  motion_planners_ompl
  PUBLIC tesseract::motion_planners
         tesseract::motion_planners_simple
         ${OMPL_LIBRARIES}
         Boost::boost
         cereal::cereal
         ${CMAKE_THREAD_LIBS_INIT})
target_compile_options(motion_planners_ompl PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(motion_planners_ompl PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(motion_planners_ompl PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(motion_planners_ompl ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(motion_planners_ompl PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  motion_planners_ompl
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(motion_planners_ompl PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                       "$<INSTALL_INTERFACE:include>")
target_include_directories(motion_planners_ompl SYSTEM PUBLIC ${OMPL_INCLUDE_DIRS})
if(OMPL_VERSION VERSION_LESS "1.4.0")
  target_compile_definitions(motion_planners_ompl PUBLIC OMPL_LESS_1_4_0=ON)
endif()

# Mark cpp header files for installation
install(DIRECTORY include/tesseract DESTINATION include COMPONENT motion_planners_ompl)

# Testing
if(TESSERACT_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# Configure Components
configure_component(
  COMPONENT motion_planners_ompl
  NAMESPACE tesseract
  TARGETS motion_planners_ompl
  DEPENDENCIES "tesseract_planning COMPONENTS motion_planners motion_planners_simple" ompl cereal)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT motion_planners_ompl
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract motion planners ompl components"
    COMPONENT_DEPENDS motion_planners motion_planners_simple
    LINUX_DEPENDS "libompl-dev"
    WINDOWS_DEPENDS "ompl")
endif()
