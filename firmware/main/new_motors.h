#include <Arduino.h>
#include <ros.h>
#include <ebobot/MotorsInfo.h>
#include <ebobot/MotorPinLayout.h>
#include <ebobot/NewMotor.h>
#include <geometry_msgs/Twist.h>
//////////////////////////
#define MAX_MOTORS 6 // 6 is absolute maximum
bool motors_debugged = false;
char motors_debug_msg[40];
//////////////////////////
ebobot::MotorsInfo motors_msg;
ros::Publisher motors_info("/motors_info", &motors_msg);
//////////////////////////
namespace Omnimotors{
  Motors * __motors[MAX_MOTORS];
  float coeff = 1;
  int num_motors = 0;
  float __motors_loop_delay = 20;
  float dtime = __motors_loop_delay / 1000.0;
  struct pin_layout{
    uint8_t encoder_pin_a;
    uint8_t encoder_pin_b;
    uint8_t pwm_pin;
    uint8_t fwd_dir_pin;
    uint8_t back_dir_pin;
  };
  class Motors{
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
    int dX;
    //////////////////////////////////////
    static void isr0(){ __motors[0]->handler();};
    static void isr1(){ __motors[1]->handler();};
    static void isr2(){ __motors[2]->handler();};
    static void isr3(){ __motors[3]->handler();};
    static void isr4(){ __motors[4]->handler();};
    static void isr5(){ __motors[5]->handler();};
    //////////////////////////////////////  
    void handler(){
      bool temp = (digitalRead(layout.encoder_pin_b) == HIGH);
      X += (temp * 1) + (!temp * -1);
    }
    static float _toRadians(float angle){
    return (angle * 6.283 / 360.0);
    }
    void termsReset(){
    last_error = 0;
    inter_term = 0;
    }
    void PID(){
      float error = targ_spd - curr_spd;
      inter_term += dtime * error;
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
        curr_spd = ddist * 1000.0 / __motors_loop_delay;
        if (stop_mot)
        {
          termsReset();
          digitalWrite(layout.pwm_pin, HIGH);
          digitalWrite(layout.fwd_dir_pin, HIGH);
          digitalWrite(layout.back_dir_pin, HIGH);
        }
        else
        {
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
          motors_msg.num = num;
          motors_msg.target_speed = targ_spd;
          motors_msg.current_speed = curr_spd;
          motors_msg.ddist = ddist;
          motors_info.publish(&motors_msg);
      }
    //////////////////////////////////////
    public:
      Motors(float angle, pin_layout mot_pin_layout,
        float _p, float _i, float _d,
        float _rad, float _ticks_per_rotation,
        float _turn_max_speed, float _max_speed){
          turn_max_speed = _turn_max_speed; max_speed = _max_speed; absolute_max_speed = _turn_max_speed + _max_speed;
          layout = mot_pin_layout;
          prop_coeff = _p; inter_coeff = _i; diff_coeff= _d;
          rad = _rad; ticks_per_rotation = _ticks_per_rotation;
          x_coeff = cos(_toRadians(angle)); y_coeff = sin(_toRadians(angle)); 
          pinMode(mot_pin_layout.encoder_pin_b, INPUT);
          pinMode(mot_pin_layout.pwm_pin, OUTPUT);
          pinMode(mot_pin_layout.fwd_dir_pin, OUTPUT);
          pinMode(mot_pin_layout.back_dir_pin, OUTPUT);
          num = num_motors;
          num_motors ++;
        }
      void change(float angle, pin_layout mot_pin_layout,
        float _p, float _i, float _d, 
        float _rad, float _ticks_per_rotation,
        float _turn_max_speed, float _max_speed){
        turn_max_speed = _turn_max_speed; max_speed = _max_speed; absolute_max_speed = _turn_max_speed + _max_speed;
        layout = mot_pin_layout;
        prop_coeff = _p; inter_coeff = _i; diff_coeff= _d; rad = _rad; ticks_per_rotation = _ticks_per_rotation;
        x_coeff = cos(_toRadians(angle)); y_coeff = sin(_toRadians(angle)); 
        pinMode(mot_pin_layout.encoder_pin_b, INPUT);
        pinMode(mot_pin_layout.pwm_pin, OUTPUT);
        pinMode(mot_pin_layout.fwd_dir_pin, OUTPUT);
        pinMode(mot_pin_layout.back_dir_pin, OUTPUT);
        }
      static void updateMotors(){
      for (int _mot = 0; _mot < num_motors; _mot++){
        Motors* curr_mot = __motors[_mot];
        curr_mot->_update();
      }
    }
      static void begin(float loop_delay){ // Must pass loop_delay
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
      static void speedCallback(const geometry_msgs::Twist &cmd_vel)
        {
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
        static void motorsSettingsCallback(const ebobot::NewMotor::Request &req, ebobot::NewMotor::Response &resp){
          if (num_motors == 0 ) return;
          if (req.motor > num_motors){
          __motors[req.motor] = new Motors ( 
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
  };
  Motors * __motors[MAX_MOTORS];
}
ros::ServiceServer<ebobot::NewMotor::Request, ebobot::NewMotor::Response> motors_settings_server("motors_settings_service", &Omnimotors::Motors::motorsSettingsCallback);
ros::Subscriber<geometry_msgs::Twist> speed_sub("cmd_vel", &Omnimotors::Motors::speedCallback);
