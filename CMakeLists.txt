cmake_minimum_required(VERSION 3.20)
project(SelfDrive CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(raylib REQUIRED)

add_library(scene STATIC
  sim/src/assets.cpp
  sim/src/road_network.cpp
  sim/src/scene.cpp
  sim/src/renderer.cpp)
target_include_directories(scene PUBLIC sim/src)
target_link_libraries(scene PUBLIC raylib)

add_executable(bake_trees sim/tools/bake_trees.cpp)
target_link_libraries(bake_trees PRIVATE scene)

add_executable(capture_frames sim/tools/capture_frames.cpp)
target_link_libraries(capture_frames PRIVATE scene)
