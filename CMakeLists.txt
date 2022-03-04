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
)
 
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
catkin_python_setup()
################################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ebobot
CATKIN_DEPENDS python-numpy map_msgs geometry_msgs nav_msgs roscpp rospy rosserial_arduino sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
  scripts/pid_setter.py 
  nodes/motors_info_callback.py 
  nodes/costmap_server.py 
  nodes/global_planer.py 
  nodes/local_planer.py
  nodes/laser_scan_callback.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

generate_messages(
  DEPENDENCIES
  std_msgs nav_msgs sensor_msgs std_msgs
)
# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   obstacle.msg
   obstacles.msg
)