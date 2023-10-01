cmake_minimum_required(VERSION 3.10)

# Set the CMake policy for the new behavior of FindPython
cmake_policy(SET CMP0148 NEW)

project(PIDController)

set(CMAKE_CXX_STANDARD 17)

# Find Python libraries using the new FindPython module
find_package(Python COMPONENTS Interpreter Development REQUIRED)
include_directories(${Python_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/utils)  # Include the utils directory

# Add the main executable
include_directories(include)
add_executable(PIDController src/main.cpp src/controller.cpp)
target_link_libraries(PIDController ${Python_LIBRARIES})

# Suppress deprecation warnings 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

# GoogleTest setup
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Test executable
add_executable(runTests tests/test_pid.cpp)
target_link_libraries(runTests ${GTEST_MAIN_LIBRARIES} pthread)

target_sources(runTests PRIVATE src/controller.cpp)