#include "FaBoPWM_PCA9685.h"
#include <Arduino.h>
#include <ros.h>
#include <ebobot/Servos.h>
#include <ebobot/ServosSettings.h>
//
ros::ServiceServer<ebobot::ServosSettings::Request, ebobot::ServosSettings::Response> servos_server("servos_service", &servo_callback);
ros::ServiceServer<ebobot::ServosSettings::Request, ebobot::ServosSettings::Response> servos_settings_server("servos_service", &servo_callback);
//
FaBoPWM servos_shield;
servos_shield.begin();
servos_shield.set_hz(1526);
//
struct Servo_mot{
    int num;
    int channel;
    int speed;
    int min_val;
    int max_val;
    int curr_val;
    bool target_state;
    bool state;
};
//
int max_num = 0;
struct Servo_mot *list[20];
//
void servo_callback(const ebobot::Servos::Request &req, ebobot::Servos::Response &resp)
{
    if (req.state){
        struct Servo_mot *servo = list[req.num];
        servo->target_state = true;
        resp.resp = servo->max_val;
    } 
    else{
        struct Servo_mot *servo = list[req.num];
        servo->target_state = false;
        resp.resp = servo->min_val;
    } 
}
//
//ebobot::ServosSettings::
void servo_settings_callback(const ebobot::ServosSettings::Request &req, ebobot::ServosSettings::Response &resp)
{
    struct Servo_mot *servo = list[req.num];
    servo->speed = req.speed;
    servo->max_val = req.max_val;
    servo->min_val = req.min_val;
    char buffer[40];
    sprintf(buffer, "Servo %d set to min %d, max%d, spd %d", req.num ,req.min_val,req.max_val, req.speed);
    resp.resp = buffer
}
ros::ServiceServer<ebobot::ServosSettings::Request, ebobot::ServosSettings::Response> server("servos_settings_service", &servo_settings_callback);
//
int servoUp(struct Servo_mot *servo){
    //nh.logerror("UP");
    int max = servo->max_val;
    int curr = servo->curr_val;
    int spd = servo->speed;
    if ((max - curr) > spd) curr += spd;
    else curr = max;
    servos_shield.set_channel_value(servo->channel,curr);
    return servo.max_val;
}
int servoDown(struct Servo_mot *servo){
    int min = servo->min_val;
    int curr = servo->curr_val;
    int spd = servo->speed;
    if (abs(min - curr) > spd) curr -= spd;
    else curr = min;
    servos_shield.set_channel_value(servo->channel,curr);
    return servo->min_val;
}
void servosUpdate(){
    for (num=0;num<max_num;num++){
        struct Servo_mot *servo = list[num]
        if (servo->target_state != servo->state){
            if (servo->target_state > servo->state) servoUp(servo);
            else servoDown(servo);
        }
        
    }
}

void createNew(int num,  int channel, int speed, int min_val, int max_val, int curr_val){
    new_servo = new struct Servo_mot{num,channel,speed,min_val,max_val,curr_val,false,false};
    list[num] = &new_servo;
    if num > max_num:
        max_num = num;
}