cmake_minimum_required(VERSION 3.13.4)

project(hTransformation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# External Libraries
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Internal Libraries
include_directories("include")

add_executable(rotation_ex src/examples/rotation_ex.cpp
  				${EINGEN3_LIBRARIES}
				)

