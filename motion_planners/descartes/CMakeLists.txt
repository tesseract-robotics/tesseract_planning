find_package(descartes_light REQUIRED COMPONENTS core)
find_package(tesseract REQUIRED COMPONENTS collision)

# Descartes Planner
add_library(
  motion_planners_descartes
  src/cereal_serialization.cpp
  src/descartes_motion_planner.cpp
  src/descartes_collision.cpp
  src/descartes_collision_edge_evaluator.cpp
  src/descartes_robot_sampler.cpp
  src/descartes_utils.cpp
  src/profile/descartes_default_move_profile.cpp
  src/profile/descartes_ladder_graph_solver_profile.cpp)
add_library(tesseract::motion_planners_descartes ALIAS motion_planners_descartes)
set_target_properties(motion_planners_descartes PROPERTIES OUTPUT_NAME tesseract_motion_planners_descartes)
target_link_libraries(
  motion_planners_descartes
  PUBLIC tesseract::motion_planners
         tesseract::motion_planners_simple
         tesseract::collision
         descartes::descartes_light
         Boost::boost
         cereal::cereal
         ${CMAKE_THREAD_LIBS_INIT})
target_compile_options(motion_planners_descartes PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(motion_planners_descartes PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(motion_planners_descartes PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(motion_planners_descartes ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(motion_planners_descartes PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  motion_planners_descartes
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(motion_planners_descartes PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                            "$<INSTALL_INTERFACE:include>")

# Mark cpp header files for installation
install(DIRECTORY include/tesseract DESTINATION include COMPONENT motion_planners_descartes)

# Testing
if(TESSERACT_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# Configure Components
configure_component(
  COMPONENT motion_planners_descartes
  NAMESPACE tesseract
  TARGETS motion_planners_descartes
  DEPENDENCIES
    "tesseract_planning COMPONENTS motion_planners motion_planners_simple"
    "descartes_light REQUIRED COMPONENTS core"
    "tesseract COMPONENTS collision"
    cereal)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT motion_planners_descartes
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract motion planners descartes components"
    COMPONENT_DEPENDS motion_planners motion_planners_simple
    LINUX_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-core"
                  "${TESSERACT_PACKAGE_PREFIX}descartes-light-core"
    WINDOWS_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-core"
                    "${TESSERACT_PACKAGE_PREFIX}descartes-light-core")
endif()
