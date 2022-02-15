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
CATKIN_DEPENDS python-numpy pypng map_msgs geometry_msgs nav_msgs roscpp rospy rosserial_arduino sensor_msgs std_msgs
#  DEPENDS system_lib
)

# catkin_install_python(PROGRAMS
#   scripts/pid_setter.py nodes/motors_info_callback.py nodes/move_base.py#nodes/costmap_server nodes/global_planer nodes/local_planer nodes/motors_info_callback 
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

