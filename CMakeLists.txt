cmake_minimum_required(VERSION 3.22)
project(suhan_robot_model_tools2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(eigenpy REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)
find_package(ament_cmake_python REQUIRED)

message(STATUS "MoveIt Core Include Directories: ${moveit_core_INCLUDE_DIRS}")


ament_export_include_directories(src)
ament_export_libraries(suhan_robot_model_tools2)
ament_export_dependencies(
  ament_cmake 
  Eigen3 
  shape_msgs
  Boost 
  eigenpy 
  ament_cmake_python
  moveit_ros_planning
  moveit_ros_planning_interface
  )

include_directories(
  src
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${eigenpy_INCLUDE_DIRS}
)

add_library(suhan_robot_model_tools2 SHARED
  src/eigen_tools/eigen_tools.cpp
  src/collision_checker/planning_scene_collision_check.cpp
)

add_library(suhan_robot_model_tools2_wrapper_cpp SHARED
  src/suhan_robot_model_tools_warpper.cpp
)


target_link_libraries(suhan_robot_model_tools2
  Eigen3::Eigen
  rclcpp::rclcpp
)

target_link_libraries(suhan_robot_model_tools2_wrapper_cpp 
  suhan_robot_model_tools2
  ${Boost_LIBRARIES}
  Python3::Python
  eigenpy::eigenpy
  Eigen3::Eigen
)

ament_target_dependencies(suhan_robot_model_tools2
  rclcpp
  shape_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
)

ament_target_dependencies(suhan_robot_model_tools2_wrapper_cpp
  rclcpp
)

set_target_properties(suhan_robot_model_tools2_wrapper_cpp 
  PROPERTIES
  PREFIX ""
  LIBRARY DESTINATION
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/srmt2
)

install(TARGETS suhan_robot_model_tools2 suhan_robot_model_tools2_wrapper_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY src/
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(srmt2)

install(TARGETS suhan_robot_model_tools2
    LIBRARY DESTINATION lib
)
install(TARGETS suhan_robot_model_tools2_wrapper_cpp
    LIBRARY DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages
)

ament_export_dependencies(ament_cmake Eigen3 Boost eigenpy ament_cmake_python)

ament_package()