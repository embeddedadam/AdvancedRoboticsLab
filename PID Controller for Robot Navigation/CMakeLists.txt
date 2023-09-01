cmake_minimum_required(VERSION 3.10)

project(PIDController)

set(CMAKE_CXX_STANDARD 17)

# Find Python libraries
find_package(PythonLibs 3 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

# Add the main executable
include_directories(include)
add_executable(PIDController src/main.cpp src/controller.cpp)
target_link_libraries(PIDController ${PYTHON_LIBRARIES})

# Suppress deprecation warnings 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

# GoogleTest setup
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Test executable
add_executable(runTests tests/test_pid.cpp)
target_link_libraries(runTests ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)

target_sources(runTests PRIVATE src/controller.cpp)
