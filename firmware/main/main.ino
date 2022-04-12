#include <Arduino.h>
#include <ros.h>
/////////////////////////
#include "TimerMs.h"
///////////////////////////
#include "test_motors.h"
#include "servos.h"
#include "kadyrov_lcd.h"
#include "pin_reader.h"
#include "start_trigger.h"
////////////////////////////Все скорости в м/с
////////////////////////////ROS init
ros::NodeHandle_<ArduinoHardware, 7, 10, 1124, 1124> nh; // recieve/publish
#define BAUD_RATE 115200
///////////////////////Loop settings
const int loop_delay = 50;
const int servo_loop_delay = 80;
TimerMs main_loop(loop_delay, 1, 0);
TimerMs servo_loop(servo_loop_delay, 1, 0);
TimerMs start_loop(200, 1, 0);
/////////////////////////////////////////////////
int freeRam () {
  extern int __heap_start, *__brkval;
  byte v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
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
  /////////////////////////////
  nh.advertise(Motors::motors_info);
  nh.subscribe(Motors::speed_sub);
  nh.advertiseService(Motors::motors_settings_server);
  nh.advertiseService(servos_server);
  nh.advertiseService(servos_settings_server);
  nh.advertiseService(lcd_server);
  nh.advertiseService(pin_reader_server);
  // Инициализация наших хедеров
  Motors::begin(loop_delay);
  nh.advertise(start_trigger);
  pinMode(_start_pin, INPUT_PULLUP);
  pinMode(_switch_pin, INPUT_PULLUP);
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
  if (main_loop.tick()){
    Motors::update_all();
  }

  if (servo_loop.tick()){
    servosUpdate();
    if (not servos_debugged){
      nh.logwarn(servos_debug);
      servos_debugged = true;
      }
    //debugServo(0);
  }
  if (start_loop.tick()){
    startUpdate();
  }
  nh.spinOnce();
}
