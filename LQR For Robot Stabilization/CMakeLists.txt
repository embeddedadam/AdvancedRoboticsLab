# Specifying the minimum version
cmake_minimum_required(VERSION 3.10)

# Project name
project(LQRStabilization)

# Set the C++ Standard
set(CMAKE_CXX_STANDARD 17)

# Include directories
include_directories(include)

# Source files
set(SOURCE_FILES src/main.cpp)

# Create the executable for the main project
add_executable(LQRStabilization ${SOURCE_FILES})

# Add Google Test
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# LQR test files
set(LQR_TEST_SOURCE_FILES tests/test_lqr.cpp)

# Test executable
add_executable(runLQRTests ${LQR_TEST_SOURCE_FILES})
target_link_libraries(runLQRTests ${GTEST_LIBRARIES} pthread)
