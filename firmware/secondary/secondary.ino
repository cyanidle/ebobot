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
#include "pin_reader.h"
#include "start_trigger.h"
////////////////////////////Все скорости в м/с
ros::NodeHandle_<ArduinoHardware, 10, 10, 1524, 1524> nh; // recieve/publish

//######################
ebobot::MotorsInfo motors_msg;
ros::Publisher motors_info("motors_info", &motors_msg);
#define BAUD_RATE 115200
//////////////////////////
#define ENCODER_PINA0 18
#define ENCODER_PINB0 31
#define EN0 12
#define FWD0 34
#define BCK0 35

#define ENCODER_PINA1 19
#define ENCODER_PINB1 38
#define EN1 8
#define FWD1 37
#define BCK1 36

#define ENCODER_PINA2 3
#define ENCODER_PINB2 49
#define EN2 9
#define FWD2 43
#define BCK2 42
///////////////////////Loop settings

const int servo_loop_delay = 150;

TimerMs servo_loop(servo_loop_delay, 1, 0);
TimerMs start_loop(200, 1, 0);

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
  nh.advertise(motors_info);
  nh.subscribe(speed_sub);
  nh.subscribe(set_pid);
  
  //
  nh.advertiseService(servos_server);
  nh.advertiseService(servos_settings_server);
  nh.advertiseService(lcd_server);
  nh.advertiseService(pin_reader_server);
  // Инициализация наших хедеров
  nh.advertise(start_trigger);
  pinMode(_start_pin, INPUT_PULLUP);
  pinMode(_switch_pin, INPUT_PULLUP);
  ///////////////////////////////
  lcdSetup();
  if (servosSetup())
    nh.logwarn("Servos shield found");
  else
    nh.logerror("Servos shield not found!");
  ////////////////////////////////
  // motors_msg.layout.data_offset = 0;
  // motors_msg.data_length = 12;
  // motors_msg.data = (float *)malloc(sizeof(float) * 12);
  ///////////////////////////////
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA0), encoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA1), encoder1, RISING); //Не забудь объявить (войну неграм)
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA2), encoder2, RISING);
  pinMode(ENCODER_PINB0, INPUT);
  pinMode(ENCODER_PINB1, INPUT);
  pinMode(ENCODER_PINB2, INPUT);
  pinMode(EN0, OUTPUT);
  pinMode(FWD0, OUTPUT);
  pinMode(BCK0, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(FWD1, OUTPUT);
  pinMode(BCK1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(FWD2, OUTPUT);
  pinMode(BCK2, OUTPUT);
  ////////
  //_allow_publish = true;
}
////////////////////////////////
void loop()
{
  if (main_loop.tick()){
    for (int mot = 0; mot < num_motors; mot++)
      update_mot(mot);
    for (int mot = 0; mot < num_motors; mot++)
    {
      motors_msg.num = mot;
      motors_msg.target_speed = targ_spd[mot];
      motors_msg.current_speed = curr_spd[mot];
      motors_msg.ddist = ddist[mot];
      motors_info.publish(&motors_msg);
    }
  }

  if (servo_loop.tick()){
    servosUpdate();
    if (not servos_debugged){
      nh.logwarn(servos_debug); 
      }
      //debugServo(0);
    servos_debugged = true;
  }
  if (start_loop.tick()){
    startUpdate();
  }
  nh.spinOnce();
}
