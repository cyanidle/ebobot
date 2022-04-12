#include "test_motors.h"

pin_layout::pin_layout(ebobot::NewMotor::pin_layout _lay){
    encoder_pin_a = _lay.encoder_pin_a;
    encoder_pin_b = _lay.encoder_pin_b;
}

Motors::Motors(pin_layout _pin_layout, float _rad, float _ticks_per_rotation,
 float _p, float _i, float _d,
 float _turn_max_speed, float _max_speed, float _angle){
    layout = _pin_layout;
    rad = _rad; ticks_per_rotation = _ticks_per_rotation;
    prop_coeff = _p; inter_coeff = _i; diff_coeff = _d;
    turn_max_speed = _turn_max_speed; max_speed = _max_speed;
    x_coeff = cos(radians(_angle)); y_coeff = sin(radians(_angle));
    ////////////
    num_motors++;
    motors[num_motors] = this;
    attachIsr(num_motors);
 }

void Motors::motorsSettingsCallback(const ebobot::NewMotor::Request &req, ebobot::NewMotor::Response &resp){
    if (req.motor > num_motors){
        if (req.motor - num_motors > 1) resp.resp = 1;
        else{
            
            Motors new_motor{}


        }
    }
    else resp.resp = 1;
}