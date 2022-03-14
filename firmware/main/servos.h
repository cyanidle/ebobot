#include <FaBoPWM_PCA9685.h>
#include <Arduino.h>
#include <ros.h>
#include <ebobot/Servos.h>
#include <ebobot/ServosSettings.h>
//
#define MAX_SERVOS 16
FaBoPWM servos_shield;
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
struct Servo_mot *ptr_list[MAX_SERVOS];
//
void servoCallback(const ebobot::Servos::Request &req, ebobot::Servos::Response &resp)
{
    if (req.state){
        struct Servo_mot *servo = ptr_list[req.num];
        servo->target_state = true;
        char buffer[40];
        sprintf(buffer, "Servo %d moving to %d, max%d, spd %d", req.num ,servo->max_val);
        resp.resp = buffer;
    } 
    else{
        struct Servo_mot *servo = ptr_list[req.num];
        servo->target_state = false;
        char buffer[40];
        sprintf(buffer, "Servo %d moving to %d, max%d, spd %d", req.num ,servo->min_val);
        resp.resp = buffer;
    } 
}
//
//ebobot::ServosSettings::
void servoSettingsCallback(const ebobot::ServosSettings::Request &req, ebobot::ServosSettings::Response &resp)
{
    if (num > MAX_SERVOS) resp.resp = "No servos left!";
    else if (num > max_num){
        createNewServo(req.num,req.channel,req.speed,req.min_val,req.max_val,0)
    }
    else{
        struct Servo_mot *servo = ptr_list[req.num];
        servo->channel = req.channel;
        servo->speed = req.speed;
        servo->max_val = req.max_val;
        servo->min_val = req.min_val;
        char buffer[40];
        sprintf(buffer, "Servo %d set to ch %d, min %d, max%d, spd %d",
        req.channel, req.num ,req.min_val,req.max_val, req.speed);
        resp.resp = buffer;
    }
    
}
//ros::ServiceServer<ebobot::ServosSettings::Request, ebobot::ServosSettings::Response> server("servos_settings_service", &servoSettingsCallback);
//
int servoUp(struct Servo_mot *servo){
    //nh.logerror("UP");
    int max = servo->max_val;
    int curr = servo->curr_val;
    int spd = servo->speed;
    if ((max - curr) > spd) curr += spd;
    else curr = max;
    servos_shield.set_channel_value(servo->channel,curr);
    return servo->max_val;
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
    for (int num=0;num<max_num;num++){
        struct Servo_mot *servo = ptr_list[num];
        if (servo->target_state != servo->state){
            if (servo->target_state > servo->state) servoUp(servo);
            else servoDown(servo);
        }
        
    }
}
void createNewServo(int num,  int channel, int speed, int min_val, int max_val, int curr_val){
    struct Servo_mot *ptr = (struct Servo_mot*) malloc(sizeof(struct Servo_mot));
    ptr_list[num] = ptr;
    ptr->num = num;
    ptr->speed = speed;
    ptr->channel = channel;
    ptr->max_val = max_val;
    ptr->min_val = min_val;
    ptr->curr_val = curr_val;
    ptr->state = false;
    ptr->target_state = false;
    //struct Servo_mot new_servo{num,channel,speed,min_val,max_val,curr_val,false,false};
    if (num > max_num) max_num = num;
}
//
ros::ServiceServer<ebobot::ServosSettings::Request, ebobot::ServosSettings::Response> servos_server("servos_service", &servoCallback);
ros::ServiceServer<ebobot::ServosSettings::Request, ebobot::ServosSettings::Response> servos_settings_server("servos_settings_service", &servoSettingsCallback);
//
