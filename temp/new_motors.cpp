#include "new_motors.h"


//////////////////////////
////////
using namespace Omnimotors;


void Motors::speedCallback(const geometry_msgs::Twist &cmd_vel){
    if (num_motors == 0 ) return;
    float x = cmd_vel.linear.x;
    float y = cmd_vel.linear.y;
    float turn = cmd_vel.angular.z;
    turn = constrain(turn, -1, 1);
    x = constrain(x, -1, 1);
    y = constrain(y, -1, 1);
    for (int _mot = 0; _mot < num_motors; _mot++){
    Motors* mot = __motors[_mot];
    if (x == 0 and y == 0 and turn == 0)
    {
        mot->stop_mot = true;
    }
    float spd = mot->x_coeff * x * mot->absolute_max_speed + mot->y_coeff * y * mot->absolute_max_speed;
    spd += turn * mot->turn_max_speed;

    // IF speed is changed radically (1/4 of max), then terms are reset
    if (abs(spd - mot->last_spds) > (mot->absolute_max_speed / 2.0))
    {
        for (int sub_mot = 0; sub_mot < num_motors; sub_mot++){
        Motors* sub = __motors[sub_mot];
        sub->termsReset();
        }
    }
    //////IF speed is less than 1 cm/second then its not considered and PID terms are reset
    if (spd < 0.01 and spd > -0.01)
    {
        mot->termsReset();
        mot->targ_spd = 0;
    }
    else
    {
        spd = constrain(spd, -mot->absolute_max_speed, mot->absolute_max_speed);
        mot->stop_mot = false;
        mot->targ_spd = spd;
        mot->last_spds = spd;
    }
    }
}
void Motors::motorsSettingsCallback(const ebobot::NewMotor::Request &req, ebobot::NewMotor::Response &resp){
    if (num_motors == 0 ) return;
    if (req.motor > num_motors){
    __motors[req.motor] = new Motors( 
    req.angle, pin_layout{
        req.pin_layout.encoder_a,
        req.pin_layout.encoder_b,
        req.pin_layout.pwm,
        req.pin_layout.fwd_dir, 
        req.pin_layout.back_dir
    },
    req.pid.P, req.pid.I, req.pid.D,
    req.wheel_rad, req.ticks_per_rotation,
    req.turn_max_speed, req.max_speed);
    if ((req.motor - num_motors) > 1){
        resp.resp = 1;
    }
    resp.resp = 0;
    }
    
    
    else{
    Motors * curr_motor = __motors[req.motor];
        curr_motor->change(
        req.angle, pin_layout{
        req.pin_layout.encoder_a,
        req.pin_layout.encoder_b,
        req.pin_layout.pwm,
        req.pin_layout.fwd_dir, 
        req.pin_layout.back_dir
    },
        req.pid.P, req.pid.I, req.pid.D,
        req.wheel_rad, req.ticks_per_rotation,
        req.turn_max_speed, req.max_speed);
    resp.resp = 0;
    }

} 
void Motors::begin(float loop_delay){ // Must pass loop_delay
__motors_loop_delay = loop_delay;
if (num_motors == 0) return;
for (int _mot = 0; _mot < num_motors; _mot++){
    dtime = __motors_loop_delay / 1000.0;
    Motors * curr = __motors[_mot];
    switch (_mot){
    case 0:
    attachInterrupt(digitalPinToInterrupt(curr->layout.encoder_pin_a),isr0, RISING);
    break;
    case 1:
    attachInterrupt(digitalPinToInterrupt(curr->layout.encoder_pin_a),isr1, RISING);
    break;
    case 2:
    attachInterrupt(digitalPinToInterrupt(curr->layout.encoder_pin_a), isr2, RISING);
    break;
    case 3:
    attachInterrupt(digitalPinToInterrupt(curr->layout.encoder_pin_a), isr3, RISING);
    break;
    case 4:
    attachInterrupt(digitalPinToInterrupt(curr->layout.encoder_pin_a), isr4, RISING);
    break;
    case 5:
    attachInterrupt(digitalPinToInterrupt(curr->layout.encoder_pin_a), isr5, RISING);
    break;
    }
}
}
void Motors::updateMotors(){
      for (int _mot = 0; _mot < num_motors; _mot++){
        Motors* curr_mot = __motors[_mot];
        curr_mot->_update();
      }
    } 
//////////////////////////////////////
    void Motors::isr0(){ __motors[0]->handler();};
    void Motors::isr1(){ __motors[1]->handler();};
    void Motors::isr2(){ __motors[2]->handler();};
    void Motors::isr3(){ __motors[3]->handler();};
    void Motors::isr4(){ __motors[4]->handler();};
    void Motors::isr5(){ __motors[5]->handler();};
//////////////////////////////////////

