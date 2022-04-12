#ifndef dor_motors_h
#define dor_motors_h
#ifndef MAX_MOTORS
#define MAX_MOTORS 6
#endif
///
#include <Arduino.h>
#include <ros.h>
#include <ebobot/NewMotor.h>
#include <geometry_msgs/Twist.h>
#include <Vector.h>
///
struct pin_layout
{
  unit8_t encoder_pin_a;
  unit8_t encoder_pin_b;
  unit8_t pwm_pin;
  unit8_t fwd_dir_pin;
  unit8_t back_dir_pin;
  pin_layout(ebobot::NewMoror::pin_layout);
};
class Motors{
    public:
    Motors(pin_layout);
    static void update_all();
    ///////////////////////
    static void isr0(){motors_list[0]->update()};
    static void isr1(){motors_list[1]->update()};
    static void isr2(){motors_list[2]->update()};
    static void isr3(){motors_list[3]->update()};
    static void isr4(){motors_list[4]->update()};
    static void isr5(){motors_list[5]->update()};
    ///
    private:
    ///
    float prop_coeff = 300; 
    float inter_coeff = 350; 
    float diff_coeff = 3;
    volatile int X = 0; 
    float targ_spd; 
    float curr_spd; 
    float last_spd;
    float ddist; 
    const float absolute_max_speed; 
    const float turn_max_speed; 
    const float max_speed;
    float ticks_per_rotation;
    float rad;
    static float radians(float);
    float x_coeff;
    float y_coeff;
    float inter_term;
    float last_error;
    void PID();
    void update();
    void termsReset();
    static void speedCallback(const geometry_msgs::Twist&);
    static void motorsSettingsCallback(const ebobot::NewMotor::Request&, ebobot::NewMotor::Response&);
};

//#define MOTORS motors_list  
#endif