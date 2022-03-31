#include <ros.h>
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <ebobot/LcdShow.h>
LiquidCrystal_I2C lcd(0x3F,16,2);
////////////////////убейте меня//////////////////
byte LT[8] = {0x07,0x0F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}; //or byte
byte UB[8] = {0x1F,0x1F,0x1F,0x00,0x00,0x00,0x00,0x00};
byte RT[8] = {0x3C,0x1E,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F};
byte LL[8] = {0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x0F,0x07};
byte LB[8] = {0x00,0x00,0x00,0x00,0x00,0x1F,0x1F,0x1F};
byte LR[8] = {0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1E,0x1C};
byte MB[8] = {0x1F,0x1F,0x1F,0x00,0x00,0x00,0x1F,0x1F};
byte block[8] = {0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F};
///////////////////////////////////////////////////
void lcdSetup(){
  //Serial.begin(9600);
  lcd.init();                      // initialize the lcd
  lcd.backlight();
  lcd.home();
  lcd.clear();
  lcd.print("Ebobot V1.1.8");
  lcd.createChar(0,LT);
  lcd.createChar(1,UB);
  lcd.createChar(2,RT);
  lcd.createChar(3,LL);
  lcd.createChar(4,LB);
  lcd.createChar(5,LR);
  lcd.createChar(6,MB);
  lcd.createChar(7,block);
}
///////////йобанная магия с StackOverflow
void reverseArray(uint16_t a[], uint8_t n)
{
    int temp;
    for (int i = 0; i < n / 2; i++)
    {
        temp = a[n - i - 1];
        a[n - i - 1] = a[i];
        a[i] = temp;
    }
}
/////////////////////////////////////////
void custom0(int x)
{ // uses segments to build the number 0
  lcd.setCursor(x,0); // set cursor to column 0, line 0 (first row)
  lcd.write(0);  // call each segment to create
  lcd.write(1);  // top half of the number
  lcd.write(2);
  lcd.setCursor(x, 1); // set cursor to colum 0, line 1 (second row)
  lcd.write(3);  // call each segment to create
  lcd.write(4);  // bottom half of the number
  lcd.write(5);
}
void custom1(int x)
{
  lcd.setCursor(x,0);
  lcd.write(1);
  lcd.write(2);
  lcd.print(" ");
  lcd.setCursor(x,1);
  lcd.write(4);
  lcd.write(7);
  lcd.write(4);
}
void custom2(int x)
{
  lcd.setCursor(x,0);
  lcd.write(6);
  lcd.write(6);
  lcd.write(2);
  lcd.setCursor(x, 1);
  lcd.write(3);
  lcd.write(4);
  lcd.write(4);
}
void custom3(int x)
{
  lcd.setCursor(x,0);
  lcd.write(6);
  lcd.write(6);
  lcd.write(2);
  lcd.setCursor(x, 1);
  lcd.write(4);
  lcd.write(4);
  lcd.write(5);
}
void custom4(int x)
{
  lcd.setCursor(x,0);
  lcd.write(3);
  lcd.write(4);
  lcd.write(7);
  lcd.setCursor(x, 1);
  lcd.print(" ");
  lcd.print(" ");
  lcd.write(7);
}
void custom5(int x)
{
  lcd.setCursor(x,0);
  lcd.write(3);
  lcd.write(6);
  lcd.write(6);
  lcd.setCursor(x, 1);
  lcd.write(4);
  lcd.write(4);
  lcd.write(5);
}
void custom6(int x)
{
  lcd.setCursor(x,0);
  lcd.write(0);
  lcd.write(6);
  lcd.write(6);
  lcd.setCursor(x, 1);
  lcd.write(3);
  lcd.write(4);
  lcd.write(5);
}
void custom7(int x)
{
  lcd.setCursor(x,0);
  lcd.write(1);
  lcd.write(1);
  lcd.write(2);
  lcd.setCursor(x, 1);
  lcd.print(" ");
  lcd.print(" ");
  lcd.write(7);
}
void custom8(int x)
{
  lcd.setCursor(x,0);
  lcd.write(0);
  lcd.write(6);
  lcd.write(2);
  lcd.setCursor(x, 1);
  lcd.write(3);
  lcd.write(4);
  lcd.write(5);
}
void custom9(int x)
{
  lcd.setCursor(x,0);
  lcd.write(0);
  lcd.write(6);
  lcd.write(2);
  lcd.setCursor(x, 1);
  lcd.print(" ");
  lcd.print(" ");
  lcd.write(7);
}

void printDigits(int digits, int x){
  switch (digits) {
  case 0:
    custom0(x);
    break;
  case 1:
    custom1(x);
    break;
  case 2:
    custom2(x);
    break;
  case 3:
    custom3(x);
    break;
  case 4:
    custom4(x);
    break;
  case 5:
    custom5(x);
    break;
  case 6:
    custom6(x);
    break;
  case 7:
    custom7(x);
    break;
  case 8:
    custom8(x);
    break;
  case 9:
    custom9(x);
    break;
  }
}
void lcdCallback(const ebobot::LcdShow::Request &req, ebobot::LcdShow::Response &resp){
    lcd.clear();
    uint16_t numb = req.num; /////входное число
    uint16_t numba = numb; //распускается на цифры в ходе подсчета
    uint8_t count = 0; //количество цифр
    //////////////подсчет цифр в числе
    scanf("%d",&numba);
    while(numba){
    numba/=10;
    count++;
    }
    ///////////////
    uint8_t n = count;
    uint16_t score[n];
    ///////////перевод числа в масив цифр
    for(uint8_t i = 0; i<n; i++){
    score[i] = numb%10;
    numb = numb/10; 
    }
    reverseArray(score, n);
    for(uint8_t i; i<n; i++){
        printDigits(score[i],i*4);}
    resp.resp = 0;
}
ros::ServiceServer<ebobot::LcdShow::Request, ebobot::LcdShow::Response> lcd_server("lcd_service", &lcdCallback);
