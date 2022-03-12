cmake_minimum_required(VERSION 2.8.3)


include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
#add_definitions(-DUSB_CON)

# generate_arduino_firmware(main
#   SRCS main.cpp ${ROS_LIB_DIR}/time.cpp
#   BOARD mega2560
#   PORT /dev/ttyACM0
# )
set(main_SKETCH  main) # Path to sketch directory
set(main_BOARD   mega2560)
set(main_PORT   /dev/ttyACM0)
set(main_SRCS ${ROS_LIB_DIR}/time.cpp)

generate_arduino_firmware(main)