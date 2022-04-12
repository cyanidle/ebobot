#ifndef dor_motors_h
#define dor_motors_h
#ifndef MAX_MOTORS
#define MAX_MOTORS 6
#endif
///
#include <Arduino.h>
#include <ros.h>
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
  pin_layout(ebobot::MotorPinLayout);
};
class Motors{
    public:
    static int num_motors = -1;
    Motors * motors[MAX_MOTORS]; 
    Motors(int, pin_layout, float, float, float, float, float, float, float, float);
    static void begin(float);
    static void update_all();
    static void speedCallback(const geometry_msgs::Twist&);
    static void motorsSettingsCallback(const ebobot::NewMotor::Request&, ebobot::NewMotor::Response&);
    static ros::Publisher motors_info("motors_info", &motors_msg);
    static ros::ServiceServer<ebobot::NewMotor::Request, ebobot::NewMotor::Response>
    motors_settings_server("motors_settings_service", &motorsSettingsCallback);
    static ros::Subscriber<geometry_msgs::Twist> speed_sub("cmd_vel", &speedCallback);
    static ebobot::MotorsInfo motors_msg;
    ////////////////////////
    private:
    ///////////////////////
    static void attachIsr(int num){
      _pin = motors[num]->layout.encoder_pin_a;
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
    };
    static void isr0(){motors_list[0]->handle()};
    static void isr1(){motors_list[1]->handle()};
    static void isr2(){motors_list[2]->handle()};
    static void isr3(){motors_list[3]->handle()};
    static void isr4(){motors_list[4]->handle()};
    static void isr5(){motors_list[5]->handle()};
    ///
    ///
    static float loop_delay = 50;
    static float dtime = loop_delay / 1000.0;
    float prop_coeff = 300; 
    float inter_coeff = 350; 
    float diff_coeff = 3;
    volatile int X = 0; 
    pin_layout layout;
    float targ_spd = 0; 
    float curr_spd = 0; 
    float last_spd = 0;
    int pwm = 0;
    float ddist = 0; 
    const float absolute_max_speed; 
    const float turn_max_speed; 
    const float max_speed;
    float ticks_per_rotation;
    float rad;
    static float radians(float deg){
      return deg/360.0 * 6.283;
    }
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
          digitalWrite(layout.pwm, HIGH);
          digitalWrite(layout.fwd_dir_pin, HIGH);
          digitalWrite(layout.back_dir_pin, HIGH);
        }

      else
      {
        PID();
        if (pwm / abs(pwm) > 0)
        {
          analogWrite(layout.pwm, abs(pwm));
          digitalWrite(layout.fwd_dir_pin, HIGH);
          digitalWrite(layout.back_dir_pin, LOW);
        }
        else
        {
          analogWrite(layout.pwm, abs(pwm)); //////////pwm varies now from -255 to 255, so we use abs
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

//#define MOTORS motors_list  
#endif
