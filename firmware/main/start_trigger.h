#include <Arduino.h>
#include <ros.h>
#include <std_srvs/Empty.h>
#include <TimerMS.h>
ros::ServiceClient<std_srvs::Empty::Request, std_srvs::Empty::Response> start_client("/ebobot/begin");
_started = false;
_start_pin = 20

void setStartPin(num){
  _start_pin = num
  pinMode(num,INPUT);
}
void startUpdate(){
  if (start_loop.tick() and not _started){
    if (digitalRead(20) == HIGH){
      _started = true
      start_client.call(std_srvs::Empty::Request req(),std_srvs::Empty::Request resp());
    }
  }
}
