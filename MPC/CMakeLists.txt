cmake_minimum_required(VERSION 3.10)

# Set the CMake policy for the new behavior of FindPython
cmake_policy(SET CMP0148 NEW)

project(MPC)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find Python libraries using the new FindPython module
find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Python_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/utils)  # Include the utils directory

# Add the main executable
include_directories(include)
add_executable(MPC src/main.cpp src/mpc.cpp)

# Linking libraries
target_link_libraries(MPC PRIVATE ${Python_LIBRARIES} osqp::osqp)

# Suppress deprecation warnings 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

# GoogleTest setup
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Test executable
add_executable(runMPCTests tests/test_mpc.cpp src/mpc.cpp)
target_link_libraries(runMPCTests ${GTEST_MAIN_LIBRARIES} pthread)

target_sources(runMPCTests PRIVATE tests/test_mpc.cpp)
