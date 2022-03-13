cmake_minimum_required(VERSION 3.0.2)
project(ebobot)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  rosserial_arduino
  sensor_msgs
  std_msgs
  genmsg 
  actionlib_msgs 
  actionlib
  rosserial_arduino
  rosserial_client
)
 
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
catkin_python_setup()

################################################
add_message_files(
   FILES
   PID.msg
   Obstacle.msg
   Obstacles.msg
  )
add_action_files(
  DIRECTORY action 
  FILES Move.action
  )
add_service_files(
    FILES
    PIDSettings.srv
    Servos.srv
    ServosSettings.srv
  )
generate_messages(
  DEPENDENCIES
  std_msgs nav_msgs sensor_msgs actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ebobot
CATKIN_DEPENDS python3-numpy map_msgs geometry_msgs 
nav_msgs roscpp rospy rosserial_arduino sensor_msgs 
std_msgs message_runtime python3-pyyaml python3-asyncio
#  DEPENDS system_lib
)
catkin_install_python(PROGRAMS
  scripts/pid_setter.py 
  nodes/motors_info_callback.py 
  nodes/costmap_server.py 
  nodes/global_planer.py 
  nodes/local_planer.py
  nodes/laser_scan_callback.py
  nodes/task_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

message(STATUS "Tvoya mama")
################# В этом месте мы опять получаем по ебалу от негров
rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

######## Ну или пытаемся скомпилировать и загрузить прошивку автоматически
rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)
########
#rosserial_add_client_target(firmware main ALL)
#rosserial_add_client_target(firmware main-upload)