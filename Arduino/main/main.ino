#include <TimerMs.h>

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
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
int ena_d[3];
////////////////////////////motors radians


float to_radians(float ang){
  return (ang * 6.283 / 360.0);
  }

int mots_angles[] = {90,210,330};
float mots_x_coeffs[] = {cos(to_radians(mots_angles[0])),cos(to_radians(mots_angles[1])),cos(to_radians(mots_angles[2]))};
float mots_y_coeffs[] = {sin(to_radians(mots_angles[0])),sin(to_radians(mots_angles[1])),sin(to_radians(mots_angles[2]))};
/////coefficients for pre counting cos



////DIR IS USED ONLY FOR SETTING PINS ON SHIELD TO NECESSARY CONFIG, WHILE SPEED IS USED FOR CALCULATING THE PWM




////DIR IS USED ONLY FOR SETTING PINS ON SHIELD TO NECESSARY CONFIG, WHILE SPEED IS USED FOR CALCULATING THE PWM
void speedCallback(const geometry_msgs::Twist& cmd_vel){
      float x = cmd_vel.linear.x;
      float y = cmd_vel.linear.y;
      float turn = cmd_vel.angular.z;
      if (x > 0.001 and x  < 0.001 and y > 0.001 and y < 0.001){
        for (int mot=0;mot<num_motors; mot++){
          dir[mot] = 0;
        }
      }
      for (int i=0;i<num_motors; i++){
        set_speed(i, x, y,turn);}}
        
ros::Subscriber<geometry_msgs::Twist> speed_sub("cmd_vel" , speedCallback);

void set_speed(int mot, float x, float y, float turn){
  
  if (x>1) x = 1;
  if (x<-1) x = -1;
  if (y>1) y = 1;
  if (y<-1) y = -1;
  if (turn>1) turn = 1;
  if (turn<-1) turn = -1;
  float spd = mots_x_coeffs[mot]*x*max_speed + mots_y_coeffs[mot]*y*max_speed;
  spd += turn * turn_max_speed;
  if (spd > mots_max_speed[mot]) spd = mots_max_speed[mot];
  if (spd < -mots_max_speed[mot]) spd = -mots_max_speed[mot];
  //////IF speed is less than 1 cm/second then its not considered
  if (spd > 0.01 or spd <- 0.01) { 
    dir[mot] = spd/abs(spd);
    targ_spd[mot] = spd;
  } 
  else{
    targ_spd[mot] = 0;
  }
  
}



//////////////////////////////////
void setup() {
  nh.initNode();
  nh.advertise(motors_info);
  nh.subscribe(speed_sub);
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
void update_dist(){
  for (int i = 0; i< num_motors; i++){
    dX[i] = X[i] - lastX[i];
    ddist[i] = dX[i] * (1.0/ticks_per_rotation) * rad * coeff;
    lastX[i] = X[i];
    dist[i] = dist[i] + ddist[i];
    update_motor(i);
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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Feedback

float dtime = (loop_delay/1000.0);


/////////Adjustable !!!!!!!!!!
float prop_coeff[] = {1,5,5};
float inter_coeff[] = {0.5,0.2,0.2};
float diff_coeff[] = {-0.1,0,0};


///////Non-Adjustable
float inter_term[] = {0,0,0};
float last_error[] = {0,0,0};

void feedback(int mot){
  float error = abs(targ_spd[mot]) - abs(curr_spd[mot]);
    inter_term[mot] += dtime * error;
    
    ena_d[mot] =  error * prop_coeff[mot] + inter_term[mot] * inter_coeff[mot] + 
    (error - last_error[mot])/ dtime * diff_coeff[mot];  

    last_error[mot] = error;
    
    if (ena_d[mot] > 255) ena_d[mot] = 255;
    if (ena_d[mot] < 0) ena_d[mot] = 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// motors update to spd and dir values
void update_motor(int mot){
    curr_spd[mot] = ddist[mot] *  1000.0/ loop_delay ;
    feedback(mot);
    switch (dir[mot]){
      case 1:     
      analogWrite(ena[mot], ena_d[mot]);
      digitalWrite(fwd[mot], HIGH);
      digitalWrite(bck[mot], LOW);
      break;
      case -1:
      analogWrite(ena[mot], ena_d[mot]);
      digitalWrite(fwd[mot], LOW);
      digitalWrite(bck[mot], HIGH);
      break;
      case 0:
      ena_d[mot] = 0;
      digitalWrite(ena[mot], HIGH);
      digitalWrite(fwd[mot], HIGH);
      digitalWrite(bck[mot], HIGH);
      break;
      default:
      break;
    }
}
////////////////////////////////////////////////////////////// 


void loop(){
  
  if (main_loop.tick()){
  update_dist();
  for (int i = 0; i< num_motors; i++){
    motors_msg.data[i*4] = targ_spd[i];
    motors_msg.data[i*4 + 1] = curr_spd[i];   
    motors_msg.data[i*4 + 2] = dist[i]; 
    motors_msg.data[i*4 + 3] = ddist[i];}
  motors_info.publish(&motors_msg);
  nh.spinOnce();}

  
}
  
  
   
