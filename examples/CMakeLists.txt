find_package(tesseract REQUIRED COMPONENTS environment collision)
find_package(trajopt_sqp REQUIRED)
find_package(trajopt_ifopt REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

list(APPEND SUPPORTED_COMPONENTS examples)

add_library(
  examples
  src/example.cpp
  src/basic_cartesian_example.cpp
  src/car_seat_example.cpp
  src/freespace_hybrid_example.cpp
  src/freespace_ompl_example.cpp
  src/glass_upright_example.cpp
  src/online_planning_example.cpp
  src/pick_and_place_example.cpp
  src/puzzle_piece_auxillary_axes_example.cpp
  src/puzzle_piece_example.cpp
  src/scene_graph_example.cpp)
add_library(tesseract::examples ALIAS examples)
set_target_properties(examples PROPERTIES OUTPUT_NAME tesseract_examples)
target_link_libraries(
  examples
  PUBLIC tesseract::environment
         tesseract::motion_planners
         tesseract::motion_planners_ompl
         tesseract::motion_planners_trajopt
         tesseract::motion_planners_trajopt_ifopt
         trajopt::trajopt_sqp
         trajopt::trajopt_ifopt
         tesseract::task_composer
         tesseract::task_composer_planning_nodes
         console_bridge::console_bridge
  PRIVATE ${PCL_LIBRARIES})
target_compile_options(examples PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(examples ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(examples PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_include_directories(examples PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                           "$<INSTALL_INTERFACE:include>")
target_include_directories(examples PRIVATE ${PCL_INCLUDE_DIRS})

macro(add_example example_name example_file)
  add_executable(${example_name} ${example_file})
  target_link_libraries(${example_name} tesseract::examples)
  target_compile_options(${example_name} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
  target_clang_tidy(${example_name} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${example_name} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
  target_include_directories(${example_name} PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                     "$<INSTALL_INTERFACE:include>")
endmacro()

add_example(tesseract_examples_basic_cartesian_example src/basic_cartesian_example_node.cpp)
add_example(tesseract_examples_car_seat_example src/car_seat_example_node.cpp)
add_example(tesseract_examples_freespace_hybrid_example src/freespace_hybrid_example_node.cpp)
add_example(tesseract_examples_freespace_ompl_example src/freespace_ompl_example_node.cpp)
add_example(tesseract_examples_glass_upright_example src/glass_upright_example_node.cpp)
add_example(tesseract_examples_online_planning_example src/online_planning_example_node.cpp)
add_example(tesseract_examples_pick_and_place_example src/pick_and_place_example_node.cpp)
add_example(tesseract_examples_puzzle_piece_auxillary_axes_example src/puzzle_piece_auxillary_axes_example_node.cpp)
add_example(tesseract_examples_puzzle_piece_example src/puzzle_piece_example_node.cpp)
add_example(tesseract_examples_scene_graph_example src/scene_graph_example_node.cpp)
add_example(tesseract_examples_benchmarks src/example_benchmarks_node.cpp)

# Mark cpp header files for installation
install(
  DIRECTORY include/tesseract
  DESTINATION include
  COMPONENT examples
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

configure_component(
  COMPONENT examples
  NAMESPACE tesseract
  TARGETS examples
          tesseract_examples_basic_cartesian_example
          tesseract_examples_car_seat_example
          tesseract_examples_freespace_hybrid_example
          tesseract_examples_freespace_ompl_example
          tesseract_examples_glass_upright_example
          tesseract_examples_online_planning_example
          tesseract_examples_pick_and_place_example
          tesseract_examples_puzzle_piece_auxillary_axes_example
          tesseract_examples_puzzle_piece_example
          tesseract_examples_scene_graph_example
          tesseract_examples_benchmarks
  DEPENDENCIES
    "tesseract COMPONENTS environment common collision"
    "tesseract_planning COMPONENTS command_language motion_planners motion_planners_simple task_composer task_composer_planning"
    trajopt_sqp
    trajopt_ifopt)

if(TESSERACT_ENABLE_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT examples
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract planning examples"
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libpcl-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-planning-command-language"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-planning-motion-planners-core"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-planning-task-composer-core"
      "${TESSERACT_PACKAGE_PREFIX}trajopt-sqp"
      "${TESSERACT_PACKAGE_PREFIX}trajopt-ifopt"
    WINDOWS_DEPENDS
      "console-bridge"
      "pcl"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-planning-command-language"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-planning-motion-planners-core"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-planning-task-composer-core"
      "${TESSERACT_PACKAGE_PREFIX}trajopt-sqp"
      "${TESSERACT_PACKAGE_PREFIX}trajopt-ifopt")
endif()

set(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS} PARENT_SCOPE)
