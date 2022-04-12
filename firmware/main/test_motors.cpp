#include "test_motors.h"
#include <ebobot/NewMotor.h>
#include <geometry_msgs/Twist.h>
#include <ebobot/MotorPinLayout.h>
//////////////////////////////////////////////////////



//char motors_debug[50];
//bool motors_debugged = true;
//////////////////////////////////////////////////////
int num_motors = -1;
Motors *motors[MAX_MOTORS]; 

void Motors::isr0(){motors[0]->handle();}
void Motors::isr1(){motors[1]->handle();}
void Motors::isr2(){motors[2]->handle();}
void Motors::isr3(){motors[3]->handle();}
void Motors::isr4(){motors[4]->handle();}
void Motors::isr5(){motors[5]->handle();}
void Motors::attachIsr(int num){
      uint8_t _pin = motors[num]->layout.encoder_pin_a;
      //sprintf(motors_debug, "Attaching isr to pin %d", _pin);
      //motors_debugged = false;
      switch (num)
      {
        case 0:
        attachInterrupt(digitalPinToInterrupt(_pin),isr0, RISING);
        break;
        case 1:
        attachInterrupt(digitalPinToInterrupt(_pin),isr1, RISING);
        break;
        case 2:
        attachInterrupt(digitalPinToInterrupt(_pin),isr2, RISING);
        break;
        case 3:
        attachInterrupt(digitalPinToInterrupt(_pin),isr3, RISING);
        break;
        case 4:
        attachInterrupt(digitalPinToInterrupt(_pin),isr4, RISING);
        break;
        case 5:
        attachInterrupt(digitalPinToInterrupt(_pin),isr5, RISING);
        break;    
      default:
        break;
      }
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
    delay(500);
    //loop_delay = _loop_delay;
    //dtime = loop_delay / 1000.0;
}
void Motors::motorsSettingsCallback(const ebobot::NewMotor::Request &req, ebobot::NewMotor::Response &resp){
    if (req.motor - num_motors > 1) resp.resp = 1;
    else{   
        pin_layout layout;
        layout.encoder_pin_a = req.pin_layout.encoder_a;
        layout.encoder_pin_b = req.pin_layout.encoder_b;
        layout.pwm_pin = req.pin_layout.pwm;
        layout.fwd_dir_pin = req.pin_layout.fwd_dir;
        layout.back_dir_pin = req.pin_layout.back_dir;
       
        
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
        float spd = motor->x_coeff * x * motor->max_speed + motor->y_coeff * y * motor->max_speed;
        spd += turn * motor->turn_max_speed;

        // IF speed is changed radically (1/4 of max), then terms are reset
        if (abs(spd - motor->last_spd) > (motor->absolute_max_speed / 2.0))
        {
        for (int sub_mot = 0; sub_mot < num_motors; sub_mot++){
            motors[sub_mot]->termsReset();
        }
        }
        //////IF speed is less than 1 cm/second then its not considered and PID terms are reset
        if (spd < 0.01 and spd > -0.01)
        {
            motor->termsReset();
            motor->targ_spd = 0;
        }
        else
        {
            spd = constrain(spd, -motor->absolute_max_speed, motor->absolute_max_speed);
            motor->stop_mot = false;
            motor->targ_spd = spd;
            motor->last_spd = spd;
        }
    }
}
