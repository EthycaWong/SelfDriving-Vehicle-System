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

add_library(driving STATIC
  sim/src/protocol.cpp
  sim/src/vehicle.cpp
  sim/src/controller.cpp
  sim/src/maneuver.cpp
  sim/src/planner.cpp
  sim/src/perception.cpp
  sim/src/vision_link.cpp
  sim/src/arduino_link.cpp
  sim/src/scorer.cpp)
target_link_libraries(driving PUBLIC scene)

add_executable(selfdrive_sim sim/src/main.cpp sim/src/dashboard.cpp)
target_link_libraries(selfdrive_sim PRIVATE driving)

add_executable(bake_trees sim/tools/bake_trees.cpp)
target_link_libraries(bake_trees PRIVATE scene)

add_executable(capture_frames sim/tools/capture_frames.cpp)
target_link_libraries(capture_frames PRIVATE scene)

enable_testing()
add_executable(test_protocol sim/tests/test_protocol.cpp)
target_link_libraries(test_protocol PRIVATE driving)
add_test(NAME protocol COMMAND test_protocol)

add_executable(test_maneuvers sim/tests/test_maneuvers.cpp)
target_link_libraries(test_maneuvers PRIVATE driving)
add_test(NAME maneuvers COMMAND test_maneuvers)
