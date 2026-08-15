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
find_package(sensor_msgs REQUIRED)
find_package(maze_geometry REQUIRED)

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
  sensor_msgs
  maze_geometry
)

install(TARGETS fase4 DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY trees/ DESTINATION share/${PROJECT_NAME}/trees)

# A planta e a rota do labirinto. Moram AQUI, e nao no sim2d: um mapa e dado da
# missao, e uma copia dele no simulador seria uma segunda planta para discordar
# da primeira. O sim2d resolve `fase4:cbr2026_fase4.yaml` pelo ament_index.
install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps)
install(DIRECTORY lotes/ DESTINATION share/${PROJECT_NAME}/lotes)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Um alvo por assunto. Nomes comecando com `test_`: o CI seleciona assim.
  foreach(alvo test_tree test_transformacao)
    ament_add_gtest(${alvo} test/${alvo}.cpp)
    if(TARGET ${alvo})
      target_include_directories(${alvo} PRIVATE include)
      ament_target_dependencies(${alvo}
        rclcpp Eigen3 fsm drone_lib stdstates stdbt behaviortree_cpp
        ament_index_cpp sensor_msgs maze_geometry)
    endif()
  endforeach()
endif()

ament_package()
