/*
  Rotation Servo Sample.
  Sample of controle FS90R(FEETECH RC Model Co.,Ltd.)
*/
#include "FaBoPWM_PCA9685.h"

FaBoPWM faboPWM;
int pos = 0;
int STOP = 0;
int FORWARD = 400;
int BACK = 250;

void setup() {
  Serial.begin(115200);
  if(faboPWM.begin()) {
    Serial.println("Find PCA9685");
    faboPWM.init(STOP);
  }
  faboPWM.set_hz(50);
  
}

void loop() {
  
  faboPWM.set_channel_value(0, STOP); 
  faboPWM.set_channel_value(1, STOP); 
  delay(1000);
  faboPWM.set_channel_value(0, FORWARD); 
  faboPWM.set_channel_value(1, BACK);
  delay(1000); 
}
