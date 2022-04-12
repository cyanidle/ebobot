#include "test_motors.h"
#include <ebobot/NewMotor.h>
#include <geometry_msgs/Twist.h>
#include <ebobot/MotorPinLayout.h>
pin_layout::pin_layout(ebobot::MotorPinLayout _lay){
    encoder_pin_a = _lay.encoder_a;
    encoder_pin_b = _lay.encoder_b;
    pwm_pin = _lay.pwm;
    fwd_dir_pin = _lay.fwd_dir;
    back_dir_pin = _lay.back_dir;
}
Motors::Motors(int num, pin_layout _pin_layout, float _rad, float _ticks_per_rotation,
 float _p, float _i, float _d,
 float _turn_max_speed, float _max_speed, float _angle){
    layout = _pin_layout;
    rad = _rad; ticks_per_rotation = _ticks_per_rotation;
    prop_coeff = _p; inter_coeff = _i; diff_coeff = _d;
    turn_max_speed = _turn_max_speed; max_speed = _max_speed;
    absolute_max_speed = turn_max_speed + max_speed;
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
void Motors::update_all(){
    for(int i = 0; i < num_motors; i++){
        motors[i]->update();
    }
}
void Motors::begin(float _loop_delay){
    loop_delay = _loop_delay;
    dtime = loop_delay / 1000.0;
}
void Motors::motorsSettingsCallback(const ebobot::NewMotor::Request &req, ebobot::NewMotor::Response &resp){
    if (req.motor - num_motors > 1) resp.resp = 1;
    else{   
        pin_layout layout;
        Motors new_motor{req.motor,layout,req.wheel_rad,req.ticks_per_rotation,
        req.pid.P,req.pid.I,req.pid.D,req.turn_max_speed,req.max_speed,
        req.angle};}
}
void Motors::speedCallback(const geometry_msgs::Twist &cmd_vel){
    float x = cmd_vel.linear.x;
    float y = cmd_vel.linear.y;
    float turn = cmd_vel.angular.z;
    turn = constrain(turn, -1, 1);
    x = constrain(x, -1, 1);
    y = constrain(y, -1, 1);
    for (int mot = 0; mot < num_motors; mot++)
    {
        Motors * motor = motors[mot]; 
        if (x == 0 and y == 0 and turn == 0)
        {
        motor->stop_mot = true;
        }
        float spd = motor->mots_x_coeffs * x * motor->max_speed + motor->mots_y_coeffs * y * motor->max_speed;
        spd += turn * motor->turn_max_speed;

        // IF speed is changed radically (1/4 of max), then terms are reset
        if (abs(spd - motor->last_spds) > (motor->absolute_max_speed / 2.0))
        {
        for (int sub_mot = 0; sub_mot < num_motors; sub_mot++){
            motors[sub_mot]->termsReset();
        }
        }
        //////IF speed is less than 1 cm/second then its not considered and PID terms are reset
        if (spd < 0.01 and spd > -0.01)
        {
            motor->termsReset(mot);
            motor->targ_spd = 0;
        }
        else
        {
            spd = constrain(spd, -motor->absolute_max_speed, motor->absolute_max_speed);
            motor->stop_mot = false;
            motor->targ_spd = spd;
            motor->last_spds = spd;
        }
    }
}
