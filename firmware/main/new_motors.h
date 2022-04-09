#include <Arduino.h>
#include <ros.h>
#include <ebobot/MotorsInfo.h>
#include <ebobot/MotorPinLayout.h>
#include <ebobot/NewMotor.h>
#include <geometry_msgs/Twist.h>
//////////////////////////
#define MAX_MOTORS 6
bool motors_debugged = false;
char motors_debug_msg[40];
//////////////////////////
struct pin_layout
{
  unit8_t encoder_pin_a;
  unit8_t encoder_pin_b;
  unit8_t pwm_pin;
  unit8_t fwd_dir_pin;
  unit8_t back_dir_pin;
  public:
    pin_layout(MotorPinLayout layout){
      encoder_pin_a = layout.encoder_a;
      encoder_pin_b = layout.encoder_b;
      pwm_pin = layout.pwm;
      fwd_dir_pin = layout.fwd_dir;
      back_dir_pin = layout.back_dir;
    }
};
// Global
ebobot::MotorsInfo motors_msg;
ros::Publisher motors_info("/motors_info", &motors_msg);
//
class Omnimotor{
  ////////////////////////////
  uint8_t num;
  float prop_coeff;
  float inter_coeff;
  float diff_coeff;
  float rad; // m
  float ticks_per_rotation;
  float x_coeff;
  float y_coeff;
  float inter_term;
  float last_error;
  pin_layout layout;
  float turn_max_speed;  /////////MUST GIVE ABSOLUTE MAX SPEED IN SUM
  float max_speed;       /////////////With headroom (<~80)
  bool stop_mot;
  float dist;
  float absolute_max_speed;
  float ddist;
  float targ_spd;
  float curr_spd;
  float last_spds;
  float pwm;
  volatile long X;
  long lastX;
  float dtime = Omnimotor::__motors_loop_delay/1000.0;
  void encoder_trigger(){
    bool temp = (digitalRead(layout.encoder_pin_b) == HIGH);
    X += (temp * 1) + (!temp * -1);
  }
  float _toRadians(float angle){
    return (angle * 6.283 / 360.0);
  }
  void termsReset(){
  last_error = 0;
  inter_term = 0;
  }
  void PID(){
    float error = motortarg_spd - curr_spd;
    inter_term += Omnimotor::dtime * error;
    pwm = error * prop_coeff + inter_term * inter_coeff -
              (error - last_error) / dtime * diff_coeff;
    inter_term = constrain(inter_term, -30000, 30000);
    last_error = error;
    pwm = constrain(pwm, -255, 255);
  }
  ////////////////////////////////
  void _update(){
      dX = X - lastX; 
      ddist = dX * (rad / ticks_per_rotation) * coeff;
      lastX = X;
      dist += ddist;
      curr_spd = ddist * 1000.0 / Omnimotor::__motors_loop_delay;
      if (stop_mot)
      {
        termsReset(motor);
        digitalWrite(layout.pwm_pin, HIGH);
        digitalWrite(layout.fwd_dir_pin, HIGH);
        digitalWrite(layout.bck_dir_pin, HIGH);
      }
      else
      {
        PID(motor);
        if (pwm / abs(pwm) > 0)
        {
          analogWrite(layout.pwm_pin, abs(pwm));
          digitalWrite(layout.fwd_dir_pin, HIGH);
          digitalWrite(layout.bck_dir_pin, LOW);
        }
        else
        {
          analogWrite(layout.pwm_pin, abs(pwm)); //////////pwm varies now from -255 to 255, so we use abs
          digitalWrite(layout.fwd_dir_pin, LOW);
          digitalWrite(layout.bck_dir_pin, HIGH);
        }

      }
        motors_msg.num = num;
        motors_msg.target_speed = targ_spd;
        motors_msg.current_speed = curr_spd;
        motors_msg.ddist = ddist;
        motors_info.publish(&motors_msg);
    }
    void init(float loop_delay){
      Omnimotor::__motors_loop_delay = loop_delay;
    }      
  //////////////////////////////
  public:
    //////// Attributes
    Omnimotor* __motors[MAX_MOTORS];
    float __motors_loop_delay;
    int __num_motors = 0;
    ////////
    Omnimotor(uint_8t num, float angle, pin_layout mot_pin_layout, float _p,
      float _i, float _d, float _rad, float _ticks_per_rotation,
      float _turn_max_speed, float _max_speed){
      turn_max_speed = _turn_max_speed; max_speed = _max_speed; absolute_max_speed = _turn_max_speed + _max_speed;
      layout = mot_pin_layout;
      prop_coeff = _p; inter_coeff = _i; diff_coeff= _d;
      rad = _rad; ticks_per_rotation = _ticks_per_rotation;
      x_coeff = cos(_toRadians(angle)); y_coeff = sin(_toRadians(angle)); 
      attachInterrupt(digitalPinToInterrupt(mot_pin_layout.encoder_pin_a), encoder_trigger, RISING);
      pinMode(mot_pin_layout.encoder_pin_b, INPUT);
      pinMode(mot_pin_layout.pwm_pin, OUTPUT);
      pinMode(mot_pin_layout.fwd_dir_pin, OUTPUT
      pinMode(mot_pin_layout.back_dir_pin, OUTPUT);
      num = Omnimotors::__num_motors;
      Omnimotors::__num_motors ++;
    }
    void change(float angle, pin_layout mot_pin_layout, float _p,
      float _i, float _d, float _rad, float _ticks_per_rotation,
      float _turn_max_speed, float _max_speed){
      turn_max_speed = _turn_max_speed; max_speed = _max_speed; absolute_max_speed = _turn_max_speed + _max_speed;
      layout = mot_pin_layout;
      prop_coeff = _p; inter_coeff = _i; diff_coeff= _d; rad = _rad; ticks_per_rotation = _ticks_per_rotation;
      x_coeff = cos(_toRadians(angle)); y_coeff = sin(_toRadians(angle)); 
      attachInterrupt(digitalPinToInterrupt(mot_pin_layout.encoder_pin_a), encoder_trigger, RISING);
      pinMode(mot_pin_layout.encoder_pin_b, INPUT);
      pinMode(mot_pin_layout.pwm_pin, OUTPUT);
      pinMode(mot_pin_layout.fwd_dir_pin, OUTPUT
      pinMode(mot_pin_layout.back_dir_pin, OUTPUT);
      }
}
void Omnimotor::updateMotors(){
      for (int _mot = 0; _mot < Omnimotor::num_motors; _mot++){
        Omnimotor* curr_mot = Omnimotor::__motors[_mot];
        curr_mot._update();
      }
    }
////////
void speedCallback(const geometry_msgs::Twist &cmd_vel)
{
  float x = cmd_vel.linear.x;
  float y = cmd_vel.linear.y;
  float turn = cmd_vel.angular.z;
  turn = constrain(turn, -1, 1);
  x = constrain(x, -1, 1);
  y = constrain(y, -1, 1);
  for (int _mot = 0; _mot < num_motors; _mot++){
    Omnimotor* mot = Omnimotor::__motors[_mot];
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
        Omnimotor* sub = Omnimotor::__motors[sub_mot];
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
///////////////////////////////////
void motorsSettingsCallback(const ebobot::NewMotor::Request &req, ebobot::NewMotor::Response &resp){
  if (num > Omnimotor::__num_motors){
  Omnimotor::__motors[req.motor] = new Omnimotor curr_mot{ 
    req.motor, //not used in change
    req.angle, pin_layout{req.pin_layout},
    req.pid.p, req.pid.i, req.pid.d,
    req.wheel_rad, req.tick_per_rotation,
    req.turn_max_speed, req.max_speed
    if ((num - Omnimotor::__num_motors) > 1){
      resp.resp = 1;
      return
    }
  };
  resp.resp = 0;
  }
  else{
    curr_motor = Omnimotor::__motors[req.motor]
    (*curr_motor).change(
      req.angle, pin_layout{req.pin_layout},
      req.pid.p, req.pid.i, req.pid.d,
      req.wheel_rad, req.tick_per_rotation,
      req.turn_max_speed, req.max_speed
    )
  }

}
ros::ServiceServer<ebobot::NewMotor::Request, ebobot::NewMotor::Response> motors_settings_server("motors_settings_service", &motorsSettingsCallback);
ros::Subscriber<geometry_msgs::Twist> speed_sub("cmd_vel", &speedCallback);