cmake_minimum_required(VERSION 3.10)

project(PIDController)

set(CMAKE_CXX_STANDARD 14)

find_package(PythonLibs 3)

include_directories(${PYTHON_INCLUDE_DIRS} include)

add_executable(PIDController src/main.cpp src/controller.cpp)

target_link_libraries(PIDController ${PYTHON_LIBRARIES})
