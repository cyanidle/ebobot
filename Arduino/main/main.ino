#include <TimerMs.h>

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>


////////////////////////////Все скорости в мм/с

////////////////////////////ROS init
ros::NodeHandle_<ArduinoHardware,10,10,1024,1532> nh; //1532 for publish and 1024 receive
std_msgs::Float32MultiArray motors_msg;
ros::Publisher motors_info("motors_info", &motors_msg);
char imu[] = "/imu";

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
int loop_delay = 50;
TimerMs main_loop(loop_delay,1,0);
///////////////////////// ENCODER

volatile long X[3];
float coeff = 1;
float rad = 0.1727; //m
float ticks_per_rotation = 330;
long dX[3];
long lastX[3];


///////////////////////// MOTORS
float turn_max_speed = 0.30;
float max_speed = 0.50;
int dir[3];
float dist[3];
float mots_max_speed[] = {0.700,0.700,0.700};
float ddist[3];
float targ_spd[3];
float curr_spd[3];
int num_motors = 3;
int fwd[] = {FWD0,FWD1,FWD2};
int bck[] = {BCK0,BCK1,BCK2};
int ena[] = {EN0,EN1,EN2};
int pwm[3];
////////////////////////////motors radians


float to_radians(float ang){
  return (ang * 6.283 / 360.0);
  }

int mots_angles[] = {90,210,330};
float mots_x_coeffs[] = {cos(to_radians(mots_angles[0])),cos(to_radians(mots_angles[1])),cos(to_radians(mots_angles[2]))};
float mots_y_coeffs[] = {sin(to_radians(mots_angles[0])),sin(to_radians(mots_angles[1])),sin(to_radians(mots_angles[2]))};
/////coefficients for pre counting cos



////DIR IS USED ONLY FOR SETTING PINS ON SHIELD TO NECESSARY CONFIG, WHILE SPEED IS USED FOR CALCULATING THE PWM



///////Non-Adjustable                                                                                      
float inter_term[] = {0,0,0};                                                                                   
float last_error[] = {0,0,0};

////DIR IS USED ONLY FOR SETTING PINS ON SHIELD INTO NECESSARY CONFIG, WHILE SPEED IS USED FOR CALCULATING THE PWM
void speedCallback(const geometry_msgs::Twist& cmd_vel){
      float x = cmd_vel.linear.x;
      float y = cmd_vel.linear.y;
      float turn = cmd_vel.angular.z;
      constrain(x,-1,1);
      constrain(y,-1,1);
      for (int mot=0;mot<num_motors; mot++){
        
        if (x == 0 and y == 0){
          last_error[mot] = 0;
          inter_term[mot] = 0;
          dir[mot] = 0;
        }
        
        float spd = mots_x_coeffs[mot]*x*max_speed + mots_y_coeffs[mot]*y*max_speed;
        spd += turn * turn_max_speed;
        if (spd > mots_max_speed[mot]) spd = mots_max_speed[mot];
        if (spd < -mots_max_speed[mot]) spd = -mots_max_speed[mot];
        //////IF speed is less than 1 cm/second then its not considered
        if (spd < 0.01 and spd > -0.01) targ_spd[mot] = 0;
        else{
          //dir[mot] = spd/abs(spd);
          targ_spd[mot] = spd;
        }
      }
      
}
ros::Subscriber<geometry_msgs::Twist> speed_sub("cmd_vel" , speedCallback);     

/////////Adjustable !!!!!!!!!!                                                                                              
float prop_coeff[] = {200,200,200};                                                                                          
float inter_coeff[] = {500,500,500};                                                                                        
float diff_coeff[] = {30,30,30};    
/////////////////////////////////////////
void setPidCallback(const std_msgs::Float32& set_pid){
  static int count;
  count++;
  int coeff = count/3;
  int mot = count%3;
  switch (coeff){
  case 0 :
    prop_coeff[mot] = set_pid.data;
    break;
  case 1:
    inter_coeff[mot] = set_pid.data;
    break;
  case 2:
    diff_coeff[mot] = set_pid.data;
    break;
  }
  if (count == 9) count =9;
}
ros::Subscriber<std_msgs::Float32> set_pid("set_pid" , setPidCallback);


//////////////////////////////////
void setup() {
  nh.initNode();
  nh.advertise(motors_info);
  nh.subscribe(speed_sub);
  nh.subscribe(set_pid);
  /*
  motors_msg.layout.dim_length = ;
  motors_msg.layout.dim[0].label = "";
  motors_msg.layout.dim[0].size = ;
  */
  motors_msg.layout.data_offset = 0;
  motors_msg.data_length = 12;
  motors_msg.data = (float *)malloc(sizeof(float)*12);
   ///////////////////////////////
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA0),encoder0,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA1),encoder1,RISING);  //Не забудь объявить (войну неграм)
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA2),encoder2,RISING);
  pinMode(ENCODER_PINB0,INPUT);
  pinMode(ENCODER_PINB1,INPUT);
  pinMode(ENCODER_PINB2,INPUT);
  pinMode(EN0,OUTPUT);
  pinMode(FWD0,OUTPUT);
  pinMode(BCK0,OUTPUT);
  pinMode(EN1,OUTPUT);
  pinMode(FWD1,OUTPUT);
  pinMode(BCK1,OUTPUT);
  pinMode(EN2,OUTPUT);
  pinMode(FWD2,OUTPUT);
  pinMode(BCK2,OUTPUT);
}




///////////////////////////////////////// Updates ALL (global num_motors) motors dists and current speeds + feedback PWM adjustments
void update_mot(int mot){
    dX[mot] = X[mot] - lastX[mot];
    ddist[mot] = dX[mot] * (1.0/ticks_per_rotation) * rad * coeff;
    lastX[mot] = X[mot];
    dist[mot] = dist[mot] + ddist[mot];
    curr_spd[mot] = ddist[mot] *  1000.0/ loop_delay;
    
    switch (dir[mot]){
      case 0:
      pwm[mot] = 0;
      digitalWrite(ena[mot], HIGH);
      digitalWrite(fwd[mot], HIGH);
      digitalWrite(bck[mot], HIGH);
      break;
      
      default:
      PID(mot);
      if (pwm[mot]/abs(pwm[mot]) > 0){
      analogWrite(ena[mot], abs(pwm[mot]));
      digitalWrite(fwd[mot], HIGH);
      digitalWrite(bck[mot], LOW); 
      }
      else {
      analogWrite(ena[mot], abs(pwm[mot])); //////////pwm varies now from -255 to 255, so we use abs
      digitalWrite(fwd[mot], LOW);
      digitalWrite(bck[mot], HIGH);  
      }
      break;
    }   
}

///////////////////////////////////////////////////////////////////////
void encoder0(){
  bool temp = (digitalRead(ENCODER_PINB0) == HIGH);
  X[0] += (temp*1) + (!temp * -1);
}
void encoder1(){
  bool temp = (digitalRead(ENCODER_PINB1) == HIGH);
  X[1] += (temp*1) + (!temp * -1);
}
void encoder2(){
  bool temp = (digitalRead(ENCODER_PINB2) == HIGH);
  X[2] += (temp*1) + (!temp * -1);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// PID
float dtime = (loop_delay/1000.0);                                                                                          
                                                                                                              


void PID(int mot){
  float error = targ_spd[mot] - curr_spd[mot];
    inter_term[mot] += dtime * error;
    pwm[mot] =  error * prop_coeff[mot] + inter_term[mot] * inter_coeff[mot] + 
    (error - last_error[mot])/ dtime * diff_coeff[mot]; 
    constrain(inter_term[mot],-30000,30000);
    last_error[mot] = error;
    constrain(pwm[mot], -255, 255);
}

////////////////////////////////////////////////////////////// 


void loop(){
  if (main_loop.tick()){
  for (int mot = 0; mot< num_motors; mot++) update_mot(mot);
  for (int mot = 0; mot< num_motors; mot++){
    motors_msg.data[mot*4] = targ_spd[mot];
    motors_msg.data[mot*4 + 1] = curr_spd[mot];   
    motors_msg.data[mot*4 + 2] = (float)pwm[mot]; 
    motors_msg.data[mot*4 + 3] = ddist[mot];}  
  }
  motors_info.publish(&motors_msg);
  nh.spinOnce();
  }  

  
  
   
