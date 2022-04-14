#include <Arduino.h>
#include <ros.h>
#include <ebobot/PinReader.h>

//char pin_reader_debug[40]; //Uncomment all for debug
//bool pin_reader_debugged = true;

void PinReaderCallback(const ebobot::PinReader::Request &req, ebobot::PinReader::Response &resp){
    //sprintf(pin_reader_debug, "Pin %d, write%d, digital%d, value-%d,pullup%d",
    // req.pin, req.write,req.digital, req.value, req.pullup);
    //pin_reader_debugged = false;
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
            resp.resp = 0;
        } 
        else{
            if (req.pullup){
            pinMode(_num, INPUT_PULLUP);}
            else pinMode(_num, INPUT);
            resp.resp = analogRead(_num);
        }
    }
    
    
}
ros::ServiceServer<ebobot::PinReader::Request, ebobot::PinReader::Response> 
pin_reader_server("pin_reader_service", &PinReaderCallback);
