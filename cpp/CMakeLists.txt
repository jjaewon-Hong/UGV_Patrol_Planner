cmake_minimum_required(VERSION 3.16)
project(ugv_patrol_planner_cpp VERSION 0.1.0 LANGUAGES CXX)

# C++17 port of the controller.
#  - ugv_core: TerrainMap, A*, return energy map, PatrolScheduler, MissionManager (no ROS dependency)
#  - controller: rclcpp node, drop-in replacement for the Python controller (built when ROS 2 is found)
# Plain build:  cmake -S cpp -B build_cpp && cmake --build build_cpp && ctest --test-dir build_cpp
# ROS 2 build:  colcon build (scripts/build.sh links this folder as package ugv_patrol_planner_cpp)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(WARNINGS -Wall -Wextra -Wpedantic -Werror)

add_library(ugv_core STATIC
  src/terrain_map.cpp src/astar.cpp src/energy.cpp src/npy.cpp src/scheduler.cpp src/mission.cpp)
target_include_directories(ugv_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_options(ugv_core PRIVATE ${WARNINGS})
set_target_properties(ugv_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(ugv_plan tools/ugv_plan.cpp)
target_link_libraries(ugv_plan PRIVATE ugv_core)

# ------------------------------------------------------------------ unit tests
option(UGV_BUILD_TESTS "Build unit tests" ON)
if(UGV_BUILD_TESTS)
  enable_testing()
  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    if(EXISTS /usr/src/googletest/CMakeLists.txt)   # Ubuntu ships the sources
      add_subdirectory(/usr/src/googletest ${CMAKE_BINARY_DIR}/googletest EXCLUDE_FROM_ALL)
    else()
      include(FetchContent)
      FetchContent_Declare(googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz)
      FetchContent_MakeAvailable(googletest)
    endif()
    add_library(GTest::gtest_main ALIAS gtest_main)
  endif()
  add_executable(test_core test/test_core.cpp test/test_mission.cpp)
  target_link_libraries(test_core PRIVATE ugv_core GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(test_core)
endif()

# ------------------------------------------------------------------ ROS 2 node
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
option(UGV_BUILD_NODE "Build the rclcpp controller node" ${rclcpp_FOUND})
if(UGV_BUILD_NODE AND ament_cmake_FOUND)
  find_package(ament_index_cpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(yaml-cpp REQUIRED)

  add_executable(controller node/controller_node.cpp node/config.cpp)
  target_link_libraries(controller ugv_core yaml-cpp)
  target_compile_options(controller PRIVATE ${WARNINGS})
  if(TARGET rclcpp::rclcpp)
    target_link_libraries(controller rclcpp::rclcpp ament_index_cpp::ament_index_cpp
      ${std_msgs_TARGETS} ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS})
  else()
    ament_target_dependencies(controller rclcpp ament_index_cpp std_msgs geometry_msgs nav_msgs)
  endif()

  install(TARGETS controller ugv_plan DESTINATION lib/${PROJECT_NAME})
  ament_package()
endif()
