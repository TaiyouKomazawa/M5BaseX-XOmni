#include <M5Stack.h>

#include "BaseX.h"

#include "WheelOmniEv3.h"
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
  static float lfv, lbv, rbv, rfv;
  static double odom_x, odom_y, odom_th;
  static int mode = 0;
  static int pwm = 0;
    
  M5.update();
  M5.Lcd.setCursor(10, 10);
  
  if(M5.BtnB.wasPressed()){
    mode += (mode < 3) ? 1 : -mode;
  }

  if(mode == 1){
    M5.Lcd.println("mode: ++");
    if(pwm < 127)
      pwm++;
  }else if(mode == 2){
    M5.Lcd.println("mode: --");
    if(pwm > -127)
      pwm--;
  }else{
    M5.Lcd.println("mode: stop");
    pwm = 0;
  }
  M5.Lcd.printf("duty:%f\r\n", pwm/127.0);
  for(size_t i = 1; i <= 4; i++){
    base_x.SetMotorSpeed(i, pwm);
  }

  lfv = LF.get_wheel_vel();
  lbv = LB.get_wheel_vel();
  rbv = RB.get_wheel_vel();
  rfv = RF.get_wheel_vel();

  //omni.get_odom(odom_x, odom_y, odom_th);

  M5.Lcd.printf("vel[mm]:\r\nLF:%.2lf\tRF:%.2lf\r\nLB:%.2lf\tRB:%.2lf\r\n", lfv, rfv, lbv, rbv);
  M5.Lcd.printf("vel_mean[mm]:%lf\r\n", (lfv+rfv+lbv+rbv)/4.0);
  delay(20);

  if(millis() > lcd_cleared+lcd_clear_int){
    M5.Lcd.clear();
    lcd_cleared = millis();
  }
}
