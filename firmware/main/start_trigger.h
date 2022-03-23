#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Bool.h>
std_msgs::Bool start_msg;
ros::Publisher start_trigger("/ebobot/begin", &start_msg);
bool _started = false;
int _start_pin = 20;

void setStartPin(int num){
  _start_pin = num;
  pinMode(num,INPUT_PULLUP);
}

void startUpdate(){
  if (not _started){
    if (digitalRead(20) == HIGH){
     _started = true;
      start_msg.data = true;
      start_trigger.publish(&start_msg);
    }
  }
}
