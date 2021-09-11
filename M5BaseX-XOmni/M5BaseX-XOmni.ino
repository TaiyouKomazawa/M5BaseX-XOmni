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

const float delta = 0.09;

float lpf(float k, float raw, float &last_lpf)
{
  double out = k * (raw - last_lpf) + last_lpf;
  last_lpf = out;
  return out;
}

void setup()
{
  M5.begin(true, false, false, true);
  Serial.begin(2000000);
  Serial.flush();
  M5.Power.begin();

  M5.Lcd.setBrightness(80);
  M5.Lcd.setTextSize(2);

  M5.Speaker.begin();
  M5.Speaker.mute();

  serial.add_frame(0, &odom_msg);
  serial.add_frame(1, &cmd_msg);
  serial.add_frame(3, &rst_msg);

  omni.begin();
}

void loop()
{
  static long lcd_cleared = millis();
  static float speed_raw_x, speed_raw_y, speed_raw_th;
  static float speed_x[2], speed_y[2], speed_th[2];
  static double odom_x, odom_y, odom_th;

  M5.update();
  M5.Lcd.setCursor(0, 0);

  serial.update();

  if(serial.read() == 0){
    speed_raw_x = -cmd_msg.data.y * 1000;
    speed_raw_y = cmd_msg.data.x * 1000;
    speed_raw_th = cmd_msg.data.th;
    if(rst_msg.data.c == 1){
      LF.set(0 ,0, 0);
      LB.set(0, 0, 0);
      RB.set(0, 0, 0);
      RF.set(0, 0, 0);
      M5.Power.reset();
    }
  }

  speed_x[1] = lpf(delta, speed_raw_x, speed_x[0]);
  speed_y[1] = lpf(delta, speed_raw_y, speed_y[0]);
  speed_th[1] = lpf(delta, speed_raw_th, speed_th[0]);

  LF.set(speed_x[1], speed_y[1], speed_th[1]);
  LB.set(speed_x[1], speed_y[1], speed_th[1]);
  RB.set(speed_x[1], speed_y[1], speed_th[1]);
  RF.set(speed_x[1], speed_y[1], speed_th[1]);

  omni.get_vel(odom_x, odom_y, odom_th);

  odom_msg.data.x = odom_y / 1000.0;
  odom_msg.data.y = -odom_x / 1000.0;
  odom_msg.data.th = odom_th;

  serial.write(0);

  M5.Lcd.printf("vel_tar[mm/s,dec/s]:\nx:%.1f\ny:%.1f\nz:%.1f\n",
                  speed_raw_x, speed_raw_y, speed_raw_th/M_PI*180);
  M5.Lcd.printf("vel_cur[mm/s,dec/s]:\nx:%.1f\ny:%.1f\nz:%.1f\n",
                  speed_x[1], speed_y[1], speed_th[1]/M_PI*180);
  M5.Lcd.printf("vel_fb [mm/s,dec/s]:\nx:%.1lf\ny:%.1lf\nz:%.1lf\n",
                  odom_x, odom_y, odom_th/M_PI*180);
  M5.Lcd.printf("time[ms]:\n%ld\n", millis());

  delay(20);

  if(millis() > lcd_cleared+lcd_clear_int){
    M5.Lcd.clear();
    lcd_cleared = millis();
  }
}
