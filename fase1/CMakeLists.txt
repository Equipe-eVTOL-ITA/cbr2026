cmake_minimum_required(VERSION 3.8)
project(fase1)

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
find_package(custom_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(vision_geometry REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(fase1 src/fase1.cpp)

ament_target_dependencies(fase1
  rclcpp
  Eigen3
  fsm
  drone_lib
  stdstates
  custom_msgs
  vision_msgs
  vision_geometry
  OpenCV
  nav_msgs
  geometry_msgs
  std_msgs
)

install(TARGETS fase1 DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # A geracao da grade e a unica logica pura desta missao, e e a que decide se
  # a arena inteira e coberta. Errar aqui nao gera erro nenhum: o drone voa
  # bonito e simplesmente nao passa por cima de uma das bases.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_grid test/test_grid.cpp)
  if(TARGET test_grid)
    target_include_directories(test_grid PRIVATE include)
    ament_target_dependencies(test_grid Eigen3 stdstates)
  endif()
endif()

ament_package()
