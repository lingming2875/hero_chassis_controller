
cmake_minimum_required(VERSION 3.0.2)
project(hero_chassis_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  hardware_interface
  pluginlib
  control_toolbox
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  std_msgs
  tf
)

generate_dynamic_reconfigure_options(
  cfg/Pid.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hero_chassis_controller
  CATKIN_DEPENDS
    roscpp
    controller_interface
    hardware_interface
    pluginlib
    control_toolbox
    dynamic_reconfigure
    geometry_msgs
    nav_msgs
    std_msgs
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/hero_chassis_controller.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  FILES hero_chassis_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
