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
//#include <Vector.h>
///

struct pin_layout
{
  uint8_t encoder_pin_a;
  uint8_t encoder_pin_b;
  uint8_t pwm_pin;
  uint8_t fwd_dir_pin;
  uint8_t back_dir_pin;
  
};
class Motors{
    public:
    Motors(int, const pin_layout, float, float, float, float, float, float, float, float);
    static void begin(float);
    static void update_all();
    static void speedCallback(const geometry_msgs::Twist&);
    static void motorsSettingsCallback(const ebobot::NewMotor::Request&, ebobot::NewMotor::Response&);
    ////////////////////////
    //static ebobot::MotorsInfo motors_msg;
    //static ros::Publisher motors_info(const char[], ebobot::MotorsInfo&);
    //static ros::Subscriber<geometry_msgs::Twist> speed_sub(const char[], void *(const geometry_msgs::Twist&));
    //static ros::ServiceServer<ebobot::NewMotor::Request, ebobot::NewMotor::Response>
    //motors_settings_server(const char[], void *(const ebobot::NewMotor::Request,ebobot::NewMotor::Response));
    private:
    ///////////////////////
    static void attachIsr(int num);
    static void isr0();
    static void isr1();
    static void isr2();
    static void isr3();
    static void isr4();
    static void isr5();
    ///
    ///
    bool stop_mot = false;
    constexpr static float loop_delay = MOTORS_LOOP_DELAY;
    constexpr static float dtime = loop_delay / 1000.0;
    float prop_coeff = 300; 
    float inter_coeff = 350; 
    float diff_coeff = 3;
    int dX;
    volatile int X = 0; 
    pin_layout layout;
    float targ_spd = 0; 
    float curr_spd = 0; 
    float last_spd = 0;
    int pwm = 0;
    float dist = 0;
    float ddist = 0; 
    float absolute_max_speed; 
    float turn_max_speed; 
    float max_speed;
    float ticks_per_rotation;
    float rad;
    float x_coeff;
    float y_coeff;
    float inter_term = 0;
    float last_error = 0;
    void handle(){
      X += digitalRead(layout.encoder_pin_b)?1:-1;
    }
    void PID(){
      float error = targ_spd - curr_spd;
      inter_term += dtime * error;
      pwm = error * prop_coeff + inter_term * inter_coeff -
                (error - last_error) / dtime * diff_coeff;
      inter_term = constrain(inter_term, -30000, 30000);
      last_error = error;
      pwm = constrain(pwm, -255, 255);
    };
    void update(){
        dX = X;
        ddist = dX * (rad / ticks_per_rotation);
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
    };
    void termsReset(){
      inter_term = 0;
      last_error = 0;
    };
    
};

#endif
