#include <Arduino.h>
#include <ros.h>
/////////////////////////
#include "TimerMs.h"
///////////////////////////
#include "servos.h"
#include "kadyrov_lcd.h"
#include "pin_reader.h"
////////////////////////////
const int servo_loop_delay = 40;
TimerMs servo_loop(servo_loop_delay, 1, 0);1
////////////////////////////ROS init
ros::NodeHandle_<ArduinoHardware, 8, 4, 1524, 1400> nh; // recieve/publish
std_msgs::Float32MultiArray motors_msg;
ros::Publisher motors_info("motors_info", &motors_msg);
/////////////////////////////////////////////////
void setup()
{
  nh.initNode();
  //nh.advertise(start_trigger);
  //
  nh.advertiseService(servos_server);
  nh.advertiseService(servos_settings_server);
  nh.advertiseService(lcd_server);
  nh.advertiseService(pin_reader_server);
  // Инициализация наших хедеров
  lcdSetup();
  if (servosSetup())
    nh.logwarn("Servos shield found");
  else
    nh.logerror("Servos shield not found!");
}
////////////////////////////////
void loop()
{
  if (servo_loop.tick())
  {
    // debugServo(0);
    servosUpdate();
    if (not servos_debugged)
      nh.loginfo(servos_debug);
    servos_debugged = true;
  }
  motors_info.publish(&motors_msg);
  nh.spinOnce();
}
