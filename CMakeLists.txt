cmake_minimum_required(VERSION 2.8.3)
project(robotx_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  usv_control_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(cfg/RobotXDiffDriveController.cfg)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES robotx_control
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs dynamic_reconfigure
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Speed Controller For VRX Robot
add_executable(robotx_diff_drive_controller_node src/robotx_diff_drive_controller.cpp src/robotx_diff_drive_controller_node.cpp)
target_link_libraries(robotx_diff_drive_controller_node ${catkin_LIBRARIES})
add_dependencies(robotx_diff_drive_controller_node usv_control_msgs_generate_messages_cpp)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS robotx_diff_drive_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)