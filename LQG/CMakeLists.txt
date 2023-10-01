cmake_minimum_required(VERSION 3.10)

project(LQGStabilization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find Python libraries
find_package(PythonLibs 3 REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PYTHON_INCLUDE_DIRS})

# Include the utils directory
include_directories(${CMAKE_SOURCE_DIR}/utils)  

# Add the main executable
include_directories(include)
add_executable(LQGStabilization src/main.cpp src/lqg.cpp)
target_link_libraries(LQGStabilization ${PYTHON_LIBRARIES})

# Suppress deprecation warnings 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

# GoogleTest setup
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Test executable
add_executable(runLQGTests tests/test_lqg.cpp src/lqg.cpp)
target_link_libraries(runLQGTests ${GTEST_MAIN_LIBRARIES} pthread)

target_sources(runLQGTests PRIVATE tests/test_lqg.cpp)