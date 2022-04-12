#include <Arduino.h>
#include <ros.h>
#include <ebobot/MotorsInfo.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
/////////////////////////
#include "TimerMs.h"
//////////////////////////// наши хедеры
#include "servos.h"
#include "kadyrov_lcd.h"
////////////////////////////Все скорости в м/с
ros::NodeHandle_<ArduinoHardware, 10, 10, 1524, 1524> nh; // recieve/publish
//######################
ebobot::MotorsInfo motors_msg;
ros::Publisher motors_info("motors_info", &motors_msg);
#define BAUD_RATE 115200
const int servo_loop_delay = 150;
TimerMs servo_loop(servo_loop_delay, 1, 0);
void debugServo(int num){
     Servo_mot* servo = ptr_list[num];
     char buffer[60];
     int target;
     sprintf(buffer, "Servo%d ch%d,min%d,max%d,spd%d,targ%d,curr%d,free%d",
     num, servo->channel, servo->min_val ,servo->max_val, servo->speed, servo->target_state, servo->curr_val,freeRam());
     nh.logwarn(buffer);
}

/////////////////////////////////////////////////
void setup()
{
  nh.getHardware()-> setBaud(BAUD_RATE);
  nh.initNode();
  nh.advertiseService(servos_server);
  nh.advertiseService(servos_settings_server);
  nh.advertiseService(lcd_server);
  ///////////////////////////////
  lcdSetup();
  if (servosSetup())
    nh.logwarn("Servos shield found");
  else
    nh.logerror("Servos shield not found!");
}
////////////////////////////////
void loop()
{
  if (servo_loop.tick()){
    servosUpdate();
    if (not servos_debugged){
      nh.logwarn(servos_debug); 
      }
    servos_debugged = true;
  }
  nh.spinOnce();
}
