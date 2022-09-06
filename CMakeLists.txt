# Copyright 2018, 2021, Gepetto team, LAAS-CNRS
#

cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_ORG loco-3d)
set(PROJECT_NAME sot-talos-balance)
set(PROJECT_DESCRIPTION
    "Collection of dynamic-graph entities aimed at implementing balance control on talos."
)
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
option(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
option(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python bindings" OFF)
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)
option(INITIALIZE_WITH_NAN "Initialize Eigen entries with NaN" OFF)
option(BUILD_ROS_PACKAGES "Build ros packages" ON)

# Project configuration
if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)
set(CUSTOM_HEADER_DIR "sot/talos_balance")
set(CXX_DISABLE_WERROR TRUE)
set(DOXYGEN_USE_MATHJAX YES)

# JRL-cmakemodule setup
include(cmake/base.cmake)
include(cmake/boost.cmake)

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX C)
project(${PROJECT_NAME} ${PROJECT_ARGS})
check_minimal_cxx_standard(11 REQUIRED)

if(INITIALIZE_WITH_NAN)
  message(STATUS "Initialize with NaN all the Eigen entries.")
  add_definitions(-DEIGEN_INITIALIZE_MATRICES_BY_NAN)
endif(INITIALIZE_WITH_NAN)

# Project dependencies

if(BUILD_PYTHON_INTERFACE)
  add_project_dependency(dynamic-graph-python 4.0.0 REQUIRED)
  string(REGEX REPLACE "-" "_" PROJECT_NAME_UNDERSCORE ${PROJECT_NAME})
  set(PY_NAME ${PROJECT_NAME_UNDERSCORE})
endif(BUILD_PYTHON_INTERFACE)

if(BUILD_TESTING)
  find_package(Boost REQUIRED COMPONENTS unit_test_framework)
endif(BUILD_TESTING)

if(Boost_VERSION GREATER 107299 OR Boost_VERSION_MACRO GREATER 107299)
  # Silence a warning about a deprecated use of boost bind by boost >= 1.73
  # without dropping support for boost < 1.73
  add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
endif()

add_project_dependency(dynamic-graph 4.4.0 REQUIRED)
add_project_dependency(sot-core REQUIRED)
add_project_dependency(eiquadprog REQUIRED)
add_project_dependency(parametric-curves REQUIRED)

# Verbosity level
if(NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))
  add_definitions(-DVP_DEBUG_MODE=${CMAKE_VERBOSITY_LEVEL} -DVP_DEBUG)
endif(NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))

set(${LIBRARY_NAME}_HEADERS
    include/${CUSTOM_HEADER_DIR}/utils/commands-helper.hh
    include/${CUSTOM_HEADER_DIR}/utils/statistics.hh
    include/${CUSTOM_HEADER_DIR}/math/fwd.hh
    include/${CUSTOM_HEADER_DIR}/robot/fwd.hh
    include/${CUSTOM_HEADER_DIR}/robot/robot-wrapper.hh
    include/${CUSTOM_HEADER_DIR}/sdk_qualisys/Network.h
    include/${CUSTOM_HEADER_DIR}/sdk_qualisys/RTPacket.h
    include/${CUSTOM_HEADER_DIR}/sdk_qualisys/RTProtocol.h
    include/${CUSTOM_HEADER_DIR}/sdk_qualisys/Markup.h)

set(${PROJECT_NAME}_SOURCES
    src/utils/statistics.cpp src/robot/robot-wrapper.cpp
    src/sdk_qualisys/Network.cpp src/sdk_qualisys/RTPacket.cpp
    src/sdk_qualisys/RTProtocol.cpp src/sdk_qualisys/Markup.cpp)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(
  ${PROJECT_NAME} PUBLIC sot-core::sot-core
                         parametric-curves::parametric-curves)

if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif(SUFFIX_SO_VERSION)

if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)

add_subdirectory(src)

if(BUILD_ROS_PACKAGES AND NOT INSTALL_PYTHON_INTERFACE_ONLY)
  add_subdirectory(ros)
endif(BUILD_ROS_PACKAGES AND NOT INSTALL_PYTHON_INTERFACE_ONLY)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif(BUILD_TESTING)

if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(FILES package.xml DESTINATION share/${PROJECT_NAME})
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)
