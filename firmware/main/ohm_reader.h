#include <Arduino.h>
#include <ros.h>
#include <ebobot/OhmReader.h>

void ohmReaderCallback(const ebobot::OhmReader::Request &req, ebobot::OhmReader::Response &resp){
    int val = analogRead(A0);
    resp.ohms = 1024 * 10000/(1024 - val);
    //        1024 * resistor/(1024 - value from pin)
}
ros::ServiceServer<ebobot::OhmReader::Request, ebobot::OhmReader::Response> ohm_reader_server("ohm_reader_service", &ohmReaderCallback);
