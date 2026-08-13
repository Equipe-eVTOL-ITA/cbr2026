cmake_minimum_required(VERSION 3.8)
project(fase4)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fsm REQUIRED)
find_package(drone_lib REQUIRED)
find_package(stdstates REQUIRED)
find_package(stdbt REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(fase4 src/fase4.cpp)

ament_target_dependencies(fase4
  rclcpp
  Eigen3
  fsm
  drone_lib
  stdstates
  stdbt
  behaviortree_cpp
  ament_index_cpp
  custom_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

install(TARGETS fase4 DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY trees/ DESTINATION share/${PROJECT_NAME}/trees)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tree test/test_tree.cpp)
  if(TARGET test_tree)
    target_include_directories(test_tree PRIVATE include)
    ament_target_dependencies(test_tree
      rclcpp Eigen3 fsm drone_lib stdstates stdbt behaviortree_cpp
      ament_index_cpp)
  endif()
endif()

ament_package()
