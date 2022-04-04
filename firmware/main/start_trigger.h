#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int8.h>
std_msgs::Int8 start_msg;
ros::Publisher start_trigger("/ebobot/begin", &start_msg);
bool _started = false;
bool _route = false;
const int _start_pin = 25;
const int _switch_pin = 29;

void startUpdate(){
  
    if (_route){
      if (digitalRead(_switch_pin) == HIGH){
        _route = false;
        start_msg.data = 1;
        start_trigger.publish(&start_msg);
      }
    }
    else{
      if (digitalRead(_switch_pin) == LOW){
        _route = true;
        start_msg.data = 2;
        start_trigger.publish(&start_msg);
      }
    }
    ///////////////////////////////////////
    if (_started){
      if (digitalRead(_start_pin) == HIGH){
        _started = false;
        start_msg.data = 3;
        start_trigger.publish(&start_msg);
      }
    }
    else{
      if (digitalRead(_start_pin) == LOW){
        _started = true;
        start_msg.data = 0;
        start_trigger.publish(&start_msg);
      }
    }
  }
  ///////////////////////////////////////
