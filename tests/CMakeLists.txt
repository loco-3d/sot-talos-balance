# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/test-paths.h.in
               ${CMAKE_CURRENT_BINARY_DIR}/test-paths.h)

# Make Boost.Test generates the main function in test cases.
add_definitions(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

set(test_distribute_deps distribute-wrench sot-core::sot-core
                         sot-core::parameter-server)

# test paths and names (without .cpp extension)
set(tests dummy test_distribute)

add_definitions(-DDEBUG=2)

foreach(test ${tests})
  add_unit_test(${test} ${test}.cpp)
  target_link_libraries(${test} PRIVATE ${PROJECT_NAME} ${${test}_deps}
                                        Boost::unit_test_framework)
  target_include_directories(
    ${test} PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
endforeach(test)

target_link_libraries(
  test_distribute PRIVATE eiquadprog::eiquadprog
                          example-robot-data::example-robot-data)

if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif(BUILD_PYTHON_INTERFACE)
