#include <Arduino.h>
#include <ros.h>
#include <ebobot/MotorsInfo.h>
#include <ebobot/MotorPinLayout.h>
#include <ebobot/NewMotor.h>
#include <geometry_msgs/Twist.h>

#define MAX_MOTORS 3 // up to 6

//######################
ebobot::MotorsInfo motors_msg;
ros::Publisher motors_info("motors_info", &motors_msg);
#define BAUD_RATE 115200
//////////////////////////
#define ENCODER_PINA0 18
#define ENCODER_PINB0 31
#define EN0 12
#define FWD0 34
#define BCK0 35

#define ENCODER_PINA1 19
#define ENCODER_PINB1 38
#define EN1 8
#define FWD1 37
#define BCK1 36

#define ENCODER_PINA2 3
#define ENCODER_PINB2 49
#define EN2 9
#define FWD2 43
#define BCK2 42
namespace Motors{
///////////////////////Loop settings
const int loop_delay = 50;
const int servo_loop_delay = 150;
TimerMs main_loop(loop_delay, 1, 0);
TimerMs servo_loop(servo_loop_delay, 1, 0);
TimerMs start_loop(200, 1, 0);

///////////////////////// ENCODER

volatile long X[MAX_MOTORS];
const float coeff = 1;
const float rad = 0.185; // m
const float ticks_per_rotation = 360;
long dX[MAX_MOTORS];
long lastX[MAX_MOTORS];

///////////////////////// MOTORS
const float turn_max_speed = 0.25;  /////////MUST GIVE ABSOLUTE MAX SPEED IN SUM
const float max_speed = 0.50;       /////////////With headroom (<~80)
bool stop_mot[MAX_MOTORS];
float dist[MAX_MOTORS];
float absolute_max_speed = 0.75;
float ddist[MAX_MOTORS];
float targ_spd[MAX_MOTORS];
float curr_spd[MAX_MOTORS];
float last_spds[MAX_MOTORS];
int num_motors = MAX_MOTORS;
int fwd[MAX_MOTORS] = {FWD0, FWD1, FWD2};
int bck[MAX_MOTORS] = {BCK0, BCK1, BCK2};
int ena[MAX_MOTORS] = {EN0, EN1, EN2};
int pwm[MAX_MOTORS];
////////////////////////////motors radians
float to_radians(float ang)
{
  return (ang * 6.283 / 360.0);
}
////////////////////////////
int mots_angles[] = {90, 210, 330};
float mots_x_coeffs[] = {cos(to_radians(mots_angles[0])), cos(to_radians(mots_angles[1])), cos(to_radians(mots_angles[2]))};
float mots_y_coeffs[] = {sin(to_radians(mots_angles[0])), sin(to_radians(mots_angles[1])), sin(to_radians(mots_angles[2]))};
/////coefficients for pre counting cos

////stop_mot IS USED ONLY FOR SETTING PINS ON SHIELD TO NECESSARY CONFIG, WHILE SPEED IS USED FOR CALCULATING THE PWM

///////Non-Adjustable
float inter_term[] = {0, 0, 0};
float last_error[] = {0, 0, 0};
//////////////////////////////////
void termsReset(int mot)
{
  last_error[mot] = 0;
  inter_term[mot] = 0;
}
//////////////////////////////////////

////

////
////stop_mot IS USED ONLY FOR SETTING PINS ON SHIELD INTO NECESSARY CONFIG, WHILE SPEED IS USED FOR CALCULATING THE PWM
void speedCallback(const geometry_msgs::Twist &cmd_vel)
{
  float x = cmd_vel.linear.x;
  float y = cmd_vel.linear.y;
  float turn = cmd_vel.angular.z;
  turn = constrain(turn, -1, 1);
  x = constrain(x, -1, 1);
  y = constrain(y, -1, 1);
  for (int mot = 0; mot < num_motors; mot++)
  {
    if (x == 0 and y == 0 and turn == 0)
    {
      stop_mot[mot] = true;
    }
    float spd = mots_x_coeffs[mot] * x * absolute_max_speed + mots_y_coeffs[mot] * y * absolute_max_speed;
    spd += turn * turn_max_speed;

    // IF speed is changed radically (1/4 of max), then terms are reset
    if (abs(spd - last_spds[mot]) > (absolute_max_speed / 2.0))
    {
      for (int sub_mot = 0; sub_mot < num_motors; sub_mot++){
      termsReset(sub_mot);
      }
    }
    //////IF speed is less than 1 cm/second then its not considered and PID terms are reset
    if (spd < 0.01 and spd > -0.01)
    {
      termsReset(mot);
      targ_spd[mot] = 0;
    }
    else
    {
      spd = constrain(spd, -absolute_max_speed, absolute_max_speed);
      stop_mot[mot] = false;
      targ_spd[mot] = spd;
      last_spds[mot] = spd;
    }
  }
}


/////////Adjustable !!!!!!!!!!
float prop_coeff[] = {280, 280, 280};
float inter_coeff[] = {300, 300, 300};
float diff_coeff[] = {3, 3, 3};

/////////////////////////////////////////
void setPidCallback(const std_msgs::Float32 &set_pid)
{
  static int pid_count = 0;
  int coeff = pid_count / 3;
  int mot = pid_count % 3;
  char buffer[35];
  sprintf(buffer, "pid:count %d,data %d", pid_count, (int)(set_pid.data));
  nh.loginfo(buffer);
  switch (coeff)
  {
  case 0:
    nh.loginfo("set prop");
    prop_coeff[mot] = set_pid.data;
    break;
  case 1:
    nh.loginfo("set inter");
    inter_coeff[mot] = set_pid.data;
    break;
  case 2:
    nh.loginfo("set diff");
    diff_coeff[mot] = set_pid.data;
    break;
  }
  pid_count++;
  if (pid_count > 8)
    pid_count = 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// PID
const float dtime = (loop_delay / 1000.0);
void PID(int mot)
{
  float error = targ_spd[mot] - curr_spd[mot];
  inter_term[mot] += dtime * error;
  pwm[mot] = error * prop_coeff[mot] + inter_term[mot] * inter_coeff[mot] -
             (error - last_error[mot]) / dtime * diff_coeff[mot];
  inter_term[mot] = constrain(inter_term[mot], -30000, 30000);
  last_error[mot] = error;
  pwm[mot] = constrain(pwm[mot], -255, 255);
}
//////////////////////////////////////// Sets pins according to PID return pwm, abs(pwm) is used, the sign determines to direction
void update_mot(int mot){
  dX[mot] = X[mot] - lastX[mot];
  ddist[mot] = dX[mot] * (rad / ticks_per_rotation) * coeff;
  lastX[mot] = X[mot];
  dist[mot] = dist[mot] + ddist[mot];
  curr_spd[mot] = ddist[mot] * 1000.0 / loop_delay;
  if (stop_mot[mot])
  {
    termsReset(mot);
    digitalWrite(ena[mot], HIGH);
    digitalWrite(fwd[mot], HIGH);
    digitalWrite(bck[mot], HIGH);
  }

  else
  {
    PID(mot);
    if (pwm[mot] / abs(pwm[mot]) > 0)
    {
      analogWrite(ena[mot], abs(pwm[mot]));
      digitalWrite(fwd[mot], HIGH);
      digitalWrite(bck[mot], LOW);
    }
    else
    {
      analogWrite(ena[mot], abs(pwm[mot])); //////////pwm varies now from -255 to 255, so we use abs
      digitalWrite(fwd[mot], LOW);
      digitalWrite(bck[mot], HIGH);
    }
  }
}
//////////////////////////////////////////////////////////////
void encoder0()
{
  (digitalRead(ENCODER_PINB0) == HIGH);
  X[0] += (temp * 1) + (!temp * -1);
}
void encoder1()
{
  bool temp = (digitalRead(ENCODER_PINB1) == HIGH);
  X[1] += (temp * 1) + (!temp * -1);
}
void encoder2()
{
  bool temp = (digitalRead(ENCODER_PINB2) == HIGH);
  X[2] += (temp * 1) + (!temp * -1);
}
void encoder3()
{
  bool temp = (digitalRead(ENCODER_PINB2) == HIGH);
  X[3] += (temp * 1) + (!temp * -1);
}
///////////////////////////////////////////////////
void begin(){
    
}


///////////////////////////////////////////////////
void motorsSettingsCallback(const ebobot::NewMotor::Request &req, ebobot::NewMotor::Response &resp){
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

///////////////////////////////////////////////////
ros::ServiceServer<ebobot::NewMotor::Request, ebobot::NewMotor::Response>
motors_settings_server("motors_settings_service", &Omnimotors::Motors::motorsSettingsCallback);
ros::Subscriber<geometry_msgs::Twist> speed_sub("cmd_vel", &speedCallback);
} // ns Motors