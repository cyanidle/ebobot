
#ifndef dor_motors_h
#define dor_motors_h

#define MAX_MOTORS 6
#define MOTORS_LOOP_DELAY 50

///
#include <Arduino.h>
#include <ros.h>
#include <ebobot/MotorsInfo.h>
#include <ebobot/NewMotor.h>
#include <ebobot/MotorPinLayout.h>
#include <geometry_msgs/Twist.h>
///
ebobot::MotorsInfo motors_msg;
ros::Publisher motors_info("motors_info", &motors_msg);
struct pin_layout
{
  uint8_t encoder_pin_a;
  uint8_t encoder_pin_b;
  uint8_t pwm_pin;
  uint8_t fwd_dir_pin;
  uint8_t back_dir_pin;
  
};
char motors_debug[60];
bool motors_debugged = true;
class Motors{
    public:
    float ddist = 0; 
    float targ_spd = 0; 
    float curr_spd = 0; 
    float last_spd = 0;
    int pwm = 0;
    float absolute_max_speed = 80; 
    float turn_max_speed = 25; 
    float max_speed = 50;
    float x_coeff;
    float y_coeff;
    bool stop_mot = false;
    float loop_delay = 50;
    float dtime = 1.0/20.0;
    pin_layout layout;
    Motors(pin_layout _pin_layout, float _rad, float _ticks_per_rotation,
     float _p, float _i, float _d,
     float _turn_max_speed, float _max_speed, float _angle){
    sprintf(motors_debug, "NewMot,pin_a:%d,pin_b:%d,rad%d,prop%d", _pin_layout.encoder_pin_a, _pin_layout.encoder_pin_b,_rad,_p);
        motors_debugged = false;
    layout = _pin_layout;
    rad = _rad; ticks_per_rotation = _ticks_per_rotation;
    prop_coeff = _p; inter_coeff = _i; diff_coeff = _d;
    turn_max_speed = _turn_max_speed; max_speed = _max_speed;
    absolute_max_speed = turn_max_speed + max_speed;
    x_coeff = cos(radians(_angle)); y_coeff = sin(radians(_angle));
    ////////////
    
};
    void handle(){
      X += digitalRead(layout.encoder_pin_b)?1:-1;
    }
    void update(){
        ddist = X * (rad / ticks_per_rotation);
        X = 0;
        dist = dist + ddist;
        curr_spd = ddist * 1000.0 / loop_delay;
        if (stop_mot)
        {
          termsReset();
          digitalWrite(layout.pwm_pin, HIGH);
          digitalWrite(layout.fwd_dir_pin, HIGH);
          digitalWrite(layout.back_dir_pin, HIGH);
        }
        else{
        PID();
        if (pwm / abs(pwm) > 0)
        {
          analogWrite(layout.pwm_pin, abs(pwm));
          digitalWrite(layout.fwd_dir_pin, HIGH);
          digitalWrite(layout.back_dir_pin, LOW);
        }
        else
        {
          analogWrite(layout.pwm_pin, abs(pwm)); //////////pwm varies now from -255 to 255, so we use abs
          digitalWrite(layout.fwd_dir_pin, LOW);
          digitalWrite(layout.back_dir_pin, HIGH);
        }
      }
    }
    void termsReset(){
      inter_term = 0;
      last_error = 0;
    }
    private:
    float prop_coeff = 300; 
    float inter_coeff = 350; 
    float diff_coeff = 3;
    volatile int X = 0; 
    float dist = 0;
    float ticks_per_rotation;
    float rad;
    float inter_term = 0;
    float last_error = 0;
    void PID(){
      float error = targ_spd - curr_spd;
      inter_term += dtime * error;
      pwm = error * prop_coeff + inter_term * inter_coeff -
                (error - last_error) / dtime * diff_coeff;
      inter_term = constrain(inter_term, -30000, 30000);
      last_error = error;
      pwm = constrain(pwm, -255, 255);
    }
      
    
    
};

//////////////////////////////////////////////////////
int num_motors = -1;
Motors *motors[MAX_MOTORS]; 

void isr0(){motors[0]->handle();}
void isr1(){motors[1]->handle();}
void isr2(){motors[2]->handle();}
void isr3(){motors[3]->handle();}
void isr4(){motors[4]->handle();}
void isr5(){motors[5]->handle();}
void attachIsr(int num){
      uint8_t _pin = motors[num]->layout.encoder_pin_a;
      uint8_t _pin_b = motors[num]->layout.encoder_pin_b;
      pinMode(_pin_b, INPUT);
      pinMode(motors[num]->layout.pwm_pin, OUTPUT);
      pinMode(motors[num]->layout.fwd_dir_pin, OUTPUT);
      pinMode(motors[num]->layout.back_dir_pin, OUTPUT);
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
    
void update_all_motors(){
    for(int i = 0; i < num_motors; i++){
        motors[i]->update();
        motors_msg.num = i;
        motors_msg.target_speed = motors[i]->targ_spd;
        motors_msg.current_speed = motors[i]->curr_spd;
        motors_msg.ddist = motors[i]->ddist;
        motors_info.publish(&motors_msg);
    }
    
    //sprintf(motors_debug, "Motor1:curr:%d,targ:%d",(int)motors[0]->curr_spd, (int)motors[0]->targ_spd);
    //    motors_debugged = false;
}
void motors_begin(float _loop_delay){
     for (int mot = 0; mot < num_motors; mot++){
      motors[mot]->loop_delay = _loop_delay;
      motors[mot]->dtime = _loop_delay/1000.0; 
    }
}
void motorsSettingsCallback(const ebobot::NewMotor::Request &req, ebobot::NewMotor::Response &resp){
    if (req.motor - num_motors > 1) resp.resp = 1;
    else{   
        pin_layout layout;
        layout.encoder_pin_a = req.pin_layout.encoder_a;
        layout.encoder_pin_b = req.pin_layout.encoder_b;
        layout.pwm_pin = req.pin_layout.pwm;
        layout.fwd_dir_pin = req.pin_layout.fwd_dir;
        layout.back_dir_pin = req.pin_layout.back_dir;
        motors[req.motor] = (Motors *) new Motors(layout,req.wheel_rad,req.ticks_per_rotation,
        req.pid.P,req.pid.I,req.pid.D,req.turn_max_speed,req.max_speed,
        req.angle);}
        
        if (req.motor > num_motors){
        num_motors++;
        motors[num_motors] = motors[req.motor];
        attachIsr(num_motors);
        } 
        else{
            motors[req.motor] = motors[req.motor];
            attachIsr(req.motor);
    }
}

void speedCallback(const geometry_msgs::Twist &cmd_vel){
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
/////////////////////////////////////////////////

ros::Subscriber<geometry_msgs::Twist> speed_sub("cmd_vel", &speedCallback);
ros::ServiceServer<ebobot::NewMotor::Request, ebobot::NewMotor::Response> motors_settings_server("motors_settings_service", &motorsSettingsCallback);













#endif
