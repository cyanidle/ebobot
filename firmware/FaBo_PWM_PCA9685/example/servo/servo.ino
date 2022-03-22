#include "FaBoPWM_PCA9685.h"

FaBoPWM faboPWM;
int pos = 0;
int MAX_VALUE = 600;
int MIN_VALUE = 300;

void setup() {
  Serial.begin(115200);
  if(faboPWM.begin()) {
    Serial.println("Find PCA9685");
    faboPWM.init(300);
  }
  faboPWM.set_hz(50);
  
}

void loop() {
  for (pos = MIN_VALUE; pos <= MAX_VALUE; pos += 1) { 
    faboPWM.set_channel_value(0, pos); 
    delay(15);                      
  }
  for (pos = 600; MAX_VALUE >= MIN_VALUE; pos -= 1) { 
    faboPWM.set_channel_value(0, pos);              
    delay(15);      
  }
}
