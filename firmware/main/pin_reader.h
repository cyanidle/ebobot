#include <Arduino.h>
#include <ros.h>
#include <ebobot/PinReader.h>

void PinReaderCallback(const ebobot::PinReader::Request &req, ebobot::PinReader::Response &resp){
    int _num = req.pin;
    if (not req.digital) _num = _num + 54;
    if (req.digital){
        if (req.pullup){
            pinMode(_num, INPUT_PULLUP);
            resp.resp = digitalRead(_num);
        } 
        else{
            if (req.write){
                pinMode(_num, OUTPUT);
                digitalWrite(_num, req.value);
                resp.resp = 0;
            } 
            else{
                pinMode(_num, INPUT);
                resp.resp = digitalRead(_num);
            } 
        }
    }
    else{
        if (req.write){
            pinMode(req.pin, OUTPUT);
            analogWrite(req.pin, req.value);
            resp.resp = 0
        } 
        else{
            pinMode(_num, INPUT);
            resp.resp = analogRead(_num);
    }
    
    
}
ros::ServiceServer<ebobot::PinReader::Request, ebobot::PinReader::Response> 
pin_reader_server("pin_reader_service", &PinReaderCallback);
