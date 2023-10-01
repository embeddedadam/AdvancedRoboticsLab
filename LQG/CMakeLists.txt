cmake_minimum_required(VERSION 3.10)

# Set the CMake policy for the new behavior of FindPython
cmake_policy(SET CMP0148 NEW)

project(LQGStabilization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find Python libraries using the new FindPython module
find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Python_INCLUDE_DIRS})

# Include the utils directory
include_directories(${CMAKE_SOURCE_DIR}/utils)  

# Add the main executable
include_directories(include)
add_executable(LQGStabilization src/main.cpp src/lqg.cpp)
target_link_libraries(LQGStabilization ${Python_LIBRARIES})

# Suppress deprecation warnings 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

# GoogleTest setup
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Test executable
add_executable(runLQGTests tests/test_lqg.cpp src/lqg.cpp)
target_link_libraries(runLQGTests ${GTEST_MAIN_LIBRARIES} pthread)

target_sources(runLQGTests PRIVATE tests/test_lqg.cpp)