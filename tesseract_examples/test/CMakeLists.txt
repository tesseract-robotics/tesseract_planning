find_package(GTest REQUIRED)

if(NOT TARGET GTest::GTest)
  add_library(GTest::GTest INTERFACE IMPORTED)
  set_target_properties(GTest::GTest PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}")
  if(${GTEST_LIBRARIES})
    set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_LIBRARIES}")
  else()
    if(MSVC)
      set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "gtest.lib")
    else()
      set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "libgtest.so")
    endif()
  endif()
endif()

if(NOT TARGET GTest::Main)
  add_library(GTest::Main INTERFACE IMPORTED)
  set_target_properties(GTest::Main PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}")
  if(${GTEST_MAIN_LIBRARIES})
    set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_MAIN_LIBRARIES}")
  else()
    if(MSVC)
      set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "gtest_main.lib")
    else()
      set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "libgtest_main.so")
    endif()
  endif()
endif()

include(GoogleTest)

add_executable(${PROJECT_NAME}_basic_cartesian_example_unit basic_cartesian_example_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_basic_cartesian_example_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}
          tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_basic_cartesian_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                            ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_basic_cartesian_example_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_basic_cartesian_example_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_basic_cartesian_example_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_gtest_discover_tests(${PROJECT_NAME}_basic_cartesian_example_unit)
add_dependencies(${PROJECT_NAME}_basic_cartesian_example_unit ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_basic_cartesian_example_unit)

add_executable(${PROJECT_NAME}_car_seat_example_unit car_seat_example_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_car_seat_example_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}
          tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_car_seat_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                     ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_car_seat_example_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_car_seat_example_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_car_seat_example_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_gtest_discover_tests(${PROJECT_NAME}_car_seat_example_unit)
add_dependencies(${PROJECT_NAME}_car_seat_example_unit ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_car_seat_example_unit)

add_executable(${PROJECT_NAME}_glass_upright_example_unit glass_upright_example_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_glass_upright_example_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}
          tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_glass_upright_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                          ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_glass_upright_example_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_glass_upright_example_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_glass_upright_example_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_gtest_discover_tests(${PROJECT_NAME}_glass_upright_example_unit)
add_dependencies(${PROJECT_NAME}_glass_upright_example_unit ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_glass_upright_example_unit)

add_executable(${PROJECT_NAME}_pick_and_place_example_unit pick_and_place_example_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_pick_and_place_example_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}
          tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_pick_and_place_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                           ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_pick_and_place_example_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_pick_and_place_example_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_pick_and_place_example_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_gtest_discover_tests(${PROJECT_NAME}_pick_and_place_example_unit)
add_dependencies(${PROJECT_NAME}_pick_and_place_example_unit ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_pick_and_place_example_unit)

add_executable(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit puzzle_piece_auxillary_axes_example_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}
          tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit
                       PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE} ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit
                           PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_gtest_discover_tests(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit)
add_dependencies(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit)

add_executable(${PROJECT_NAME}_puzzle_piece_example_unit puzzle_piece_example_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_puzzle_piece_example_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}
          tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_puzzle_piece_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                         ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_puzzle_piece_example_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_puzzle_piece_example_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_puzzle_piece_example_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_gtest_discover_tests(${PROJECT_NAME}_puzzle_piece_example_unit)
add_dependencies(${PROJECT_NAME}_puzzle_piece_example_unit ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_puzzle_piece_example_unit)

add_executable(${PROJECT_NAME}_scene_graph_example_unit scene_graph_example_unit.cpp)
target_link_libraries(
  ${PROJECT_NAME}_scene_graph_example_unit
  PRIVATE GTest::GTest
          GTest::Main
          ${PROJECT_NAME}
          tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_scene_graph_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                        ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_scene_graph_example_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_scene_graph_example_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_scene_graph_example_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_gtest_discover_tests(${PROJECT_NAME}_scene_graph_example_unit)
add_dependencies(${PROJECT_NAME}_scene_graph_example_unit ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_scene_graph_example_unit)
