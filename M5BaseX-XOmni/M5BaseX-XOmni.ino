#include <M5Stack.h>

#include <BaseX.h>

#include <WheelOmniEv3.h>
#include "Omni4.h"

#include <SerialBridge.hpp>
#include <InoHardwareSerial.hpp>

#include "Vector3.h"
#include "Uint8Data.h"

#define D_OMNI_MM   48 //mm
#define O_2_OMNI_MM  (132.1*(1/sqrt(2))) //mm

BASE_X base_x = BASE_X();

WheelOmniEv3 LF( -O_2_OMNI_MM, O_2_OMNI_MM,  -M_PI*3/4.0,  D_OMNI_MM, 1, base_x);
WheelOmniEv3 LB( -O_2_OMNI_MM, -O_2_OMNI_MM, -M_PI*1/4.0,  D_OMNI_MM, 2, base_x);
WheelOmniEv3 RB( O_2_OMNI_MM,  -O_2_OMNI_MM, M_PI*1/4.0,   D_OMNI_MM, 3, base_x);
WheelOmniEv3 RF( O_2_OMNI_MM,  O_2_OMNI_MM,  M_PI*3/4.0,   D_OMNI_MM, 4, base_x);

Omni4 omni(LF, LB, RB, RF);

int lcd_clear_int = 5000; //ms

SerialDev *dev = new InoHardwareSerial(&Serial);
SerialBridge serial(dev);

Vector3 odom_msg;
Vector3 cmd_msg;
Uint8Data rst_msg;

void setup()
{
  M5.begin(true, false, true, true);
  M5.Power.begin();

  M5.Lcd.setBrightness(200);
  M5.Lcd.setTextSize(2);

  serial.add_frame(0, &odom_msg);
  serial.add_frame(1, &cmd_msg);
  serial.add_frame(3, &rst_msg);

  omni.begin();
}

void loop()
{
  static long lcd_cleared = millis();
  static float speed_x, speed_y, speed_th;
  static double odom_x, odom_y, odom_th;

  M5.update();
  M5.Lcd.setCursor(10, 10);

  serial.update();

  if(serial.read() == 0){
    speed_x = cmd_msg.data.x * 1000;
    speed_y = cmd_msg.data.y * 1000;
    speed_th = cmd_msg.data.th * 1000;
    if (rst_msg.data.c == 1){
      M5.Power.reset();
    }
  }

  LF.set(speed_x, speed_y, speed_th);
  LB.set(speed_x, speed_y, speed_th);
  RB.set(speed_x, speed_y, speed_th);
  RF.set(speed_x, speed_y, speed_th);

  omni.get_vel(odom_x, odom_y, odom_th);

  M5.Lcd.printf("speed[mm,dec]:\n %.1f\n %.1f\n %.1f\n",speed_x, speed_y, speed_th/M_PI*180);
  M5.Lcd.printf("pose [mm,dec]:\n %.1lf\n %.1lf\n %.1lf\n",odom_x, odom_y, odom_th/M_PI*180);

  odom_msg.data.x = odom_x;
  odom_msg.data.y = odom_y;
  odom_msg.data.th = odom_th;

  serial.write(0);

  delay(20);

  if(millis() > lcd_cleared+lcd_clear_int){
    M5.Lcd.clear();
    lcd_cleared = millis();
  }
}
