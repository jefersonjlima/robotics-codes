cmake_minimum_required(VERSION 3.13.4)

project(hTransformation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# External Libraries
find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED )
include_directories(${PYTHON_INCLUDE_DIRS})

# Internal Libraries
include_directories("include")

if(EXAMPLE EQUAL 1 OR COMPILE_ALL)
  add_subdirectory(src/examples/1)
endif()


if(COMPILE_TESTS OR COMPILE_ALL)
  add_subdirectory(tests)
endif() 

