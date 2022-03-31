#include <FaBoPWM_PCA9685.h>
#include <Arduino.h>
#include <ros.h>
#include <ebobot/Servos.h>
#include <ebobot/ServosSettings.h>
//
//ros::NodeHandle_<ArduinoHardware, 10, 10, 1024, 1532> nh;
#define MAX_SERVOS 8
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
    int8_t curr_val;
    int8_t target_state;
    };
//
char servos_debug[50] = "Servos ready for debug!";
bool servos_debugged = true;
int max_num = -1;
Servo_mot *ptr_list[MAX_SERVOS];
//

void servoCallback(const ebobot::Servos::Request &req, ebobot::Servos::Response &resp)
{
    Servo_mot *servo = ptr_list[req.num];
    servo->target_state = req.state;
    //
    sprintf(servos_debug, "Servo moved! serv %d: state %d (%d)", req.num ,servo->target_state,
    servo->min_val + (servo->max_val - servo->min_val) * servo->target_state / 100);
    servos_debugged = false;
    //
    resp.resp = 0;
}
void createNewServo(int num,  int channel, int speed, int min_val, int max_val, int curr_val){
    Servo_mot *ptr = malloc(sizeof(Servo_mot));
    ptr_list[num] = ptr;
    ptr->num = num;
    if (speed) ptr->speed = speed;
    else ptr->speed = 1;
    ptr->channel = channel;
    ptr->max_val = max_val;
    ptr->min_val = min_val;
    ptr->curr_val = curr_val;
    ptr->target_state = min_val;
    sprintf(servos_debug, "New serv%d:ch%d,spd%d,min%d,max%d,size%d",
         ptr->num, ptr->channel ,ptr->speed, ptr->min_val, ptr->max_val,sizeof(Servo_mot));
    servos_debugged = false;
    //Servo_mot new_servo{num,channel,speed,min_val,max_val,curr_val,false,false};
    if (num > max_num) max_num = num;
}
void servoSettingsCallback(const ebobot::ServosSettings::Request &req, ebobot::ServosSettings::Response &resp)
{
    
    

    
    if (req.num > MAX_SERVOS) resp.resp = 1;
    else if (req.num > max_num){
        if ((req.num - max_num) > 1){
            sprintf(servos_debug, "Error! servos should be init from 0 one by one");
            servos_debugged = false;
            resp.resp = 1;
        }
        else{
        createNewServo(req.num,req.channel,req.speed,req.min_val,req.max_val,0);
        resp.resp = 0;
        }
        
    }
    else{
        Servo_mot *servo = ptr_list[req.num];
        servo->channel = req.channel;
        if (req.speed) servo->speed = req.speed;
        else servo->speed = 1;
        servo->max_val = req.max_val;
        servo->min_val = req.min_val;
        sprintf(servos_debug, "Servo set! serv %d: ch %d, spd %d, min %d, max %d",
         req.num, servo->channel ,servo->speed, servo->min_val, servo->max_val);
        servos_debugged = false;
        resp.resp = 0;
    }
    
}
void servoUp(Servo_mot *servo){
    int target = servo->min_val + (servo->max_val - servo->min_val) * servo->target_state / 100;
    if (abs(target - servo->curr_val) > servo->speed) servo->curr_val += servo->speed;
    else {
        servo->curr_val = target;
    }
    servos_shield.set_channel_value(servo->channel,servo->curr_val);
}
void servoDown(Servo_mot *servo){
    int target = servo->min_val + (servo->max_val - servo->min_val) * servo->target_state / 100;
    if (abs(target - servo->curr_val) > servo->speed) servo->curr_val -= servo->speed;
    else {
        servo->curr_val = target;
    }
    servos_shield.set_channel_value(servo->channel,servo->curr_val);
}
void servosUpdate(){
    for (int num=0;num<max_num;num++){
        Servo_mot *servo = ptr_list[num];
        if (servo->target_state > servo->curr_val) servoUp(servo); 
        else if (servo->target_state < servo->curr_val) servoDown(servo);
        }       
    }

//
ros::ServiceServer<ebobot::Servos::Request, ebobot::Servos::Response> servos_server("servos_service", &servoCallback);
ros::ServiceServer<ebobot::ServosSettings::Request, ebobot::ServosSettings::Response> servos_settings_server("servos_settings_service", &servoSettingsCallback);
//
