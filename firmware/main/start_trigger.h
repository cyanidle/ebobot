#include <Arduino.h>
#include <ros.h>
#include <std_srvs/Empty.h>
ros::ServiceClient<std_srvs::Empty::Request, std_srvs::Empty::Response> start_client("/ebobot/begin");
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
      std_srvs::Empty::Request req;
      std_srvs::Empty::Response resp;
      start_client.call(req,resp);
    }
  }
}
