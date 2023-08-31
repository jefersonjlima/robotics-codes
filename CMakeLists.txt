cmake_minimum_required(VERSION 3.13.4)

project(robotics-codes)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(WITH_PLOT_GRAPH "Option to use the Matplotlib" ON)
option(WITH_SAVE_OUTPUT_CSV "Option to save output as CSV" ON)

# External Libraries
find_package(Eigen3 REQUIRED)
if(WITH_PLOT_GRAPH)
  message(STATUS "Enable MATPLOTLIBCPP library")
 find_package(PythonLibs REQUIRED)
 include_directories(${PYTHON_INCLUDE_DIRS})
 add_definitions(-DUSE_MATPLOTLIB)
endif()

if(WITH_SAVE_OUTPUT_CSV)
  add_definitions(-DSAVE_OUTPUT_CSV)
endif()

# Internal Libraries
include_directories("include")

if(EXAMPLE EQUAL 1 OR COMPILE_ALL)
  add_subdirectory(src/examples/1)
endif()


if(COMPILE_TESTS OR COMPILE_ALL)
  add_subdirectory(tests)
endif() 

