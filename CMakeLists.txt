cmake_minimum_required(VERSION 3.1)
project(gjk_demo)
set(CMAKE_CXX_STANDARD 14)

file(GLOB_RECURSE sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)
file(GLOB_RECURSE headers ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

add_executable(${PROJECT_NAME} ${sources} ${headers})
# add_executable(${PROJECT_NAME} main.cc)
