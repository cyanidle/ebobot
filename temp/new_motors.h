#include <Arduino.h>
#include <ros.h>
#include <ebobot/MotorsInfo.h>
#include <ebobot/MotorPinLayout.h>
#include <ebobot/NewMotor.h>
#include <geometry_msgs/Twist.h>
//////////////////////////
inline ebobot::MotorsInfo motors_msg;
inline ros::Publisher motors_info("/motors_info", &motors_msg);
//////////////////////////
#define MAX_MOTORS 6 // 6 is absolute maximum
namespace Omnimotors{
  inline float coeff = 1;
  inline int num_motors = 0;
  inline float __motors_loop_delay = 20;
  inline float dtime = __motors_loop_delay / 1000.0;
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
    static void isr0();
    static void isr1();
    static void isr2();
    static void isr3();
    static void isr4();
    static void isr5();
    //////////////////////////////////////
    public:
      static void updateMotors();
      static void begin(float);
      static void motorsSettingsCallback(const ebobot::NewMotor::Request &, ebobot::NewMotor::Response &);
      static void speedCallback(const geometry_msgs::Twist &);
      /////////////////////
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
  };
  inline Motors * __motors[MAX_MOTORS];
  /////////////////////////////////////
  
}
  inline ros::ServiceServer<ebobot::NewMotor::Request, ebobot::NewMotor::Response> motors_settings_server("motors_settings_service", &Omnimotors::Motors::motorsSettingsCallback);
  inline ros::Subscriber<geometry_msgs::Twist> speed_sub("cmd_vel", &Omnimotors::Motors::speedCallback);
