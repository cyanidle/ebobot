#include <Arduino.h>
#include <ros.h>
#include <ebobot/MotorsInfo.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
/////////////////////////
#include "TimerMs.h"
////
#include "pin_reader.h"
#include "start_trigger.h"
////////////////////////////ROS init
ros::NodeHandle_<ArduinoHardware, 10, 10, 2124, 1624> nh; // recieve/publish

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
///////////////////////Loop settings
const int loop_delay = 50;
const int servo_loop_delay = 150;
TimerMs main_loop(loop_delay, 1, 0);
TimerMs start_loop(200, 1, 0);
///////////////////////// ENCODER
volatile long X[3];
const float coeff = 1;
const float rad = 0.185; // m
const float ticks_per_rotation = 360;
long dX[3];
long lastX[3];
///////////////////////// MOTORS
const float turn_max_speed = 0.25;  /////////MUST GIVE ABSOLUTE MAX SPEED IN SUM
const float max_speed = 0.50;       /////////////With headroom (<~80)
bool stop_mot[3];
float dist[3];
float absolute_max_speed = 0.75;
float ddist[3];
float targ_spd[3];
float curr_spd[3];
float last_spds[3];
int num_motors = 3;
int fwd[] = {FWD0, FWD1, FWD2};
int bck[] = {BCK0, BCK1, BCK2};
int ena[] = {EN0, EN1, EN2};
int pwm[3];
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
ros::Subscriber<geometry_msgs::Twist> speed_sub("cmd_vel", &speedCallback);

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
ros::Subscriber<std_msgs::Float32> set_pid("set_pid", &setPidCallback);
//////////////////////////////////////// Updates ALL (global num_motors) motors dists and current speeds + feedback PWM adjustments

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
///////////
void encoder0()
{
  bool temp = (digitalRead(ENCODER_PINB0) == HIGH);
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

/////////////////////////////////////////////////
void setup()
{
  nh.getHardware()->setBaud(BAUD_RATE);
  nh.initNode();
  nh.advertiseService(pin_reader_server);
  // Инициализация наших хедеров
  nh.advertise(start_trigger);
  pinMode(_start_pin, INPUT_PULLUP);
  pinMode(_switch_pin, INPUT_PULLUP);
  //////////////////////////////
  nh.advertise(motors_info);
  nh.subscribe(speed_sub);
  nh.subscribe(set_pid);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA0), encoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA1), encoder1, RISING); //Не забудь объявить (войну неграм)
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA2), encoder2, RISING);
  pinMode(ENCODER_PINB0, INPUT);
  pinMode(ENCODER_PINB1, INPUT);
  pinMode(ENCODER_PINB2, INPUT);
  pinMode(EN0, OUTPUT);
  pinMode(FWD0, OUTPUT);
  pinMode(BCK0, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(FWD1, OUTPUT);
  pinMode(BCK1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(FWD2, OUTPUT);
  pinMode(BCK2, OUTPUT);
}
////////////////////////////////
void loop()
{
  if (main_loop.tick()){
    for (int mot = 0; mot < num_motors; mot++)
      update_mot(mot);
    for (int mot = 0; mot < num_motors; mot++)
    {
      motors_msg.num = mot;
      motors_msg.target_speed = targ_spd[mot];
      motors_msg.current_speed = curr_spd[mot];
      motors_msg.ddist = ddist[mot];
      motors_info.publish(&motors_msg);
      nh.spinOnce();
    }
  }
  if (start_loop.tick()){
    startUpdate();
  }
  nh.spinOnce();
}
