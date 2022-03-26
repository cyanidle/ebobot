#include <FaBoPWM_PCA9685.h>
#include <Arduino.h>
#include <ros.h>
#include <ebobot/Servos.h>
#include <ebobot/ServosSettings.h>
//
//ros::NodeHandle_<ArduinoHardware, 10, 10, 1024, 1532> nh;
#define MAX_SERVOS 16
FaBoPWM servos_shield;
bool servosSetup(){
  if (servos_shield.begin()){
    servos_shield.init(300);
    servos_shield.set_hz(1516);
    return true;
  }
}
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
char servos_debug[50] = "Servos ready for debug!";
bool servos_debugged = false;
int max_num = 0;
struct Servo_mot *ptr_list[MAX_SERVOS];
//

void servoCallback(const ebobot::Servos::Request &req, ebobot::Servos::Response &resp)
{
    if (req.state){
        struct Servo_mot *servo = ptr_list[req.num];
        servo->target_state = true;
        //char buffer[40];
        //sprintf(buffer, "Servo %d moving to %d, max%d, spd %d", req.num ,servo->max_val);
        resp.resp = 1;
    } 
    else{
        struct Servo_mot *servo = ptr_list[req.num];
        servo->target_state = false;
        //char buffer[40];
        //sprintf(buffer, "Servo %d moving to %d, max%d, spd %d", req.num ,servo->min_val);
        resp.resp = 0;
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
void servoSettingsCallback(const ebobot::ServosSettings::Request &req, ebobot::ServosSettings::Response &resp)
{
    
    //char buffer[50];
    sprintf(servos_debug, "Servo set! serv %d: ch %d, spd %d, min %d, max %d", req.num ,req.channel, req.speed, req.min_val, req.max_val);
    servos_debugged = false;
    //nh.loginfo(buffer);

    
    if (req.num > MAX_SERVOS) resp.resp = 1;
    else if (req.num > max_num){
        createNewServo(req.num,req.channel,req.speed,req.min_val,req.max_val,0);
        resp.resp = 0;
    }
    else{
        struct Servo_mot *servo = ptr_list[req.num];
        servo->channel = req.channel;
        servo->speed = req.speed;
        servo->max_val = req.max_val;
        servo->min_val = req.min_val;
        resp.resp = 0;
    }
    
}
void servoUp(struct Servo_mot *servo){
    //nh.logerror("UP");
    int max = servo->max_val;
    //int curr = servo->curr_val;
    if ((max - servo->curr_val) > servo->speed) servo->curr_val += servo->speed;
    else {
        servo->curr_val = max;
        servo->state = true;
    }
    servos_shield.set_channel_value(servo->channel,servo->curr_val);
    //return servo->max_val;
}
void servoDown(struct Servo_mot *servo){
    int min = servo->min_val;
    //int curr = servo->curr_val;
    if (abs(min - servo->curr_val) > servo->speed) servo->curr_val -= servo->speed;
    else {
        servo->curr_val = min;
        servo->state = false;
    }
    servos_shield.set_channel_value(servo->channel,servo->curr_val);
    //return servo->min_val;
}
void servosUpdate(){
    for (int num=0;num<max_num;num++){
        struct Servo_mot *servo = ptr_list[num];
        if (servo->target_state) servoUp(servo); 
        else servoDown(servo);
        }       
    }

//
ros::ServiceServer<ebobot::Servos::Request, ebobot::Servos::Response> servos_server("servos_service", &servoCallback);
ros::ServiceServer<ebobot::ServosSettings::Request, ebobot::ServosSettings::Response> servos_settings_server("servos_settings_service", &servoSettingsCallback);
//
