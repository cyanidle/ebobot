#include "test_motors.h"
#include <ebobot/NewMotor.h>
#include <geometry_msgs/Twist.h>

pin_layout::pin_layout(ebobot::NewMotor::pin_layout _lay){
    encoder_pin_a = _lay.encoder_pin_a;
    encoder_pin_b = _lay.encoder_pin_b;
}

Motors::Motors(int num, pin_layout _pin_layout, float _rad, float _ticks_per_rotation,
 float _p, float _i, float _d,
 float _turn_max_speed, float _max_speed, float _angle, bool change){
    layout = _pin_layout;
    rad = _rad; ticks_per_rotation = _ticks_per_rotation;
    prop_coeff = _p; inter_coeff = _i; diff_coeff = _d;
    turn_max_speed = _turn_max_speed; max_speed = _max_speed;
    x_coeff = cos(radians(_angle)); y_coeff = sin(radians(_angle));
    ////////////
    if (num > num_motors){
        num_motors++;
        motors[num_motors] = this;
        attachIsr(num_motors);
    } 
    else{
        motors[num] = this;
        attachIsr(num);
    }
}

void Motors::motorsSettingsCallback(const ebobot::NewMotor::Request &req, ebobot::NewMotor::Response &resp){
    if (req.motor - num_motors > 1) resp.resp = 1;
    else{   
        Motors new_motor{};


        }
}
void Motors::speedCallback(const geometry_msgs::Twist &cmd){
    
}
