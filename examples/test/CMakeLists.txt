find_gtest()

add_executable(examples_basic_cartesian_example_unit basic_cartesian_example_unit.cpp)
target_link_libraries(examples_basic_cartesian_example_unit PRIVATE GTest::GTest GTest::Main examples)
target_compile_options(examples_basic_cartesian_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                     ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(examples_basic_cartesian_example_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(examples_basic_cartesian_example_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(examples_basic_cartesian_example_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_gtest_discover_tests(examples_basic_cartesian_example_unit)
add_dependencies(examples_basic_cartesian_example_unit examples)

add_executable(examples_car_seat_example_unit car_seat_example_unit.cpp)
target_link_libraries(examples_car_seat_example_unit PRIVATE GTest::GTest GTest::Main examples)
target_compile_options(examples_car_seat_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                              ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(examples_car_seat_example_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(examples_car_seat_example_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(examples_car_seat_example_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_gtest_discover_tests(examples_car_seat_example_unit)
add_dependencies(examples_car_seat_example_unit examples)

add_executable(examples_glass_upright_example_unit glass_upright_example_unit.cpp)
target_link_libraries(examples_glass_upright_example_unit PRIVATE GTest::GTest GTest::Main examples)
target_compile_options(examples_glass_upright_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                   ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(examples_glass_upright_example_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(examples_glass_upright_example_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(examples_glass_upright_example_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_gtest_discover_tests(examples_glass_upright_example_unit)
add_dependencies(examples_glass_upright_example_unit examples)

add_executable(examples_pick_and_place_example_unit pick_and_place_example_unit.cpp)
target_link_libraries(examples_pick_and_place_example_unit PRIVATE GTest::GTest GTest::Main examples)
target_compile_options(examples_pick_and_place_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                    ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(examples_pick_and_place_example_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(examples_pick_and_place_example_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(examples_pick_and_place_example_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_gtest_discover_tests(examples_pick_and_place_example_unit)
add_dependencies(examples_pick_and_place_example_unit examples)

add_executable(examples_puzzle_piece_auxillary_axes_example_unit puzzle_piece_auxillary_axes_example_unit.cpp)
target_link_libraries(examples_puzzle_piece_auxillary_axes_example_unit PRIVATE GTest::GTest GTest::Main examples)
target_compile_options(examples_puzzle_piece_auxillary_axes_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                                 ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(examples_puzzle_piece_auxillary_axes_example_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(examples_puzzle_piece_auxillary_axes_example_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(examples_puzzle_piece_auxillary_axes_example_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_gtest_discover_tests(examples_puzzle_piece_auxillary_axes_example_unit)
add_dependencies(examples_puzzle_piece_auxillary_axes_example_unit examples)

add_executable(examples_puzzle_piece_example_unit puzzle_piece_example_unit.cpp)
target_link_libraries(examples_puzzle_piece_example_unit PRIVATE GTest::GTest GTest::Main examples)
target_compile_options(examples_puzzle_piece_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                  ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(examples_puzzle_piece_example_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(examples_puzzle_piece_example_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(examples_puzzle_piece_example_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_gtest_discover_tests(examples_puzzle_piece_example_unit)
add_dependencies(examples_puzzle_piece_example_unit examples)

add_executable(examples_scene_graph_example_unit scene_graph_example_unit.cpp)
target_link_libraries(examples_scene_graph_example_unit PRIVATE GTest::GTest GTest::Main examples)
target_compile_options(examples_scene_graph_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                 ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(examples_scene_graph_example_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(examples_scene_graph_example_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(examples_scene_graph_example_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_gtest_discover_tests(examples_scene_graph_example_unit)
add_dependencies(examples_scene_graph_example_unit examples)
