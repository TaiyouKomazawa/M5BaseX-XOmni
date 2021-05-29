#include <M5Stack.h>

#include <BaseX.h>

#include <WheelOmniEv3.h>
#include "Omni4.h"

#define D_OMNI_MM   48 //mm
#define O_2_OMNI_MM  (132.1*(1/sqrt(2))) //mm

BASE_X base_x = BASE_X();

WheelOmniEv3 LF( -O_2_OMNI_MM, O_2_OMNI_MM,  -M_PI*3/4.0,  D_OMNI_MM, 2, base_x);
WheelOmniEv3 LB( -O_2_OMNI_MM, -O_2_OMNI_MM, -M_PI*1/4.0,  D_OMNI_MM, 3, base_x);
WheelOmniEv3 RB( O_2_OMNI_MM,  -O_2_OMNI_MM, M_PI*1/4.0,   D_OMNI_MM, 4, base_x);
WheelOmniEv3 RF( O_2_OMNI_MM,  O_2_OMNI_MM,  M_PI*3/4.0,   D_OMNI_MM, 1, base_x);

Omni4 omni(LF, LB, RB, RF);

int lcd_clear_int = 5000; //ms

void setup()
{
  M5.begin(true, false, true, true);
  M5.Power.begin();

  M5.Lcd.setBrightness(200);
  M5.Lcd.setTextSize(2);


  omni.begin();
}

void loop()
{
  static long lcd_cleared = millis();
  static float speed_x, speed_y, speed_th;
  static double odom_x, odom_y, odom_th;
  static int mode = 0;


  /*int32_t encoder[4];
  for (size_t i = 1; i < 5; i++)
  {
    encoder[i - 1] = base_x.GetEncoderValue(i);
  }*/
    
  M5.update();
  M5.Lcd.setCursor(10, 10);
  
  if(M5.BtnB.wasPressed()){
    mode += (mode < 3) ? 1 : -mode;
  }

  if(mode == 0){
    M5.Lcd.print(" x-speed-set\r\n");
    if(M5.BtnC.wasPressed()){
      speed_x += 5;
    }else if(M5.BtnA.wasPressed()){
      speed_x -= 5;
    }
  }else if(mode == 1){
    M5.Lcd.print(" y-speed-set\r\n");
    if(M5.BtnC.wasPressed()){
      speed_y += 5;
    }else if(M5.BtnA.wasPressed()){
      speed_y -= 5;
    }
  }else{
    M5.Lcd.print("th-speed-set\r\n");
    if(M5.BtnC.wasPressed()){
      speed_th += M_PI/32;
    }else if(M5.BtnA.wasPressed()){
      speed_th -= M_PI/32;
    }
  }

  LF.set(speed_x, speed_y, speed_th);
  LB.set(speed_x, speed_y, speed_th);
  RB.set(speed_x, speed_y, speed_th);
  RF.set(speed_x, speed_y, speed_th);

  omni.get_vel(odom_x, odom_y, odom_th);

  M5.Lcd.printf("speed:\n %.1f\n %.1f\n %.1f\n",speed_x, speed_y, speed_th/M_PI*180);
  M5.Lcd.printf("pose:\n %.1lf\n %.1lf\n %.1lf\n",odom_x, odom_y, odom_th/M_PI*180);
  //M5.Lcd.printf("encoder:\r\n 1: %d\t2: %d\r\n 3: %d\t4: %d\r\n", encoder[0], encoder[1], encoder[2], encoder[3]);
  delay(20);

  if(millis() > lcd_cleared+lcd_clear_int){
    M5.Lcd.clear();
    lcd_cleared = millis();
  }
}
