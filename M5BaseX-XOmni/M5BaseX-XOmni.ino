#define M5STACK_MPU6886

#include <M5Stack.h>

#include <BaseX.h>

#include <WheelOmniEv3.h>
#include "Omni4.h"

#include <SerialBridge.hpp>
#include <InoHardwareSerial.hpp>

#include "bmm150.h"
#include "bmm150_defs.h"

#include "Vector3.h"
#include "Uint8Data.h"
#include "Axis9Sensor.h"

//#define DEBUG_MODE

#define D_OMNI_MM   48 //mm
#define O_WHEEL_MM  97.8 //mm

#define DEG2RAD(x) (x *= M_PI / 180.0)
#define G2A(x)     (x *= 9.80)

BASE_X base_x = BASE_X();

WheelOmniEv3 LF( O_WHEEL_MM, -M_PI*3/4.0,  D_OMNI_MM, 3, base_x);
WheelOmniEv3 LB( O_WHEEL_MM, -M_PI*1/4.0,  D_OMNI_MM, 4, base_x);
WheelOmniEv3 RB( O_WHEEL_MM, M_PI*1/4.0,   D_OMNI_MM, 1, base_x);
WheelOmniEv3 RF( O_WHEEL_MM, M_PI*3/4.0,   D_OMNI_MM, 2, base_x);

Omni4 omni(LF, LB, RB, RF);

const int ctrl_interval = 1000/60.0; //50 hz

const int timeout_interval = 1000; //1 sec

SerialDev *dev = new InoHardwareSerial(&Serial);
SerialBridge serial(dev, 1024);

BMM150 bmm = BMM150();

Vector3 odom_msg;
Vector3 cmd_msg;
Uint8Data rst_msg;
Axis9Sensor msense_msg;

void setup()
{
  M5.begin(true, false, false, true);
  Serial.begin(2000000);
  Serial.flush();
  M5.Power.begin();
  if(!M5.Power.canControl()) {
    //can't control.
    return;
  }
  M5.Power.setCharge(false);

  M5.Lcd.setBrightness(80);
  M5.Lcd.setTextSize(2);

  if(bmm.initialize() == BMM150_E_ID_NOT_CONFORM){
    M5.Lcd.setCursor(0, 10);
    M5.Lcd.println("BMM150 not found!");
    while(1){}
  }

  M5.IMU.Init();

  M5.Speaker.begin();
  M5.Speaker.mute();

  serial.add_frame(0, &odom_msg);
  serial.add_frame(1, &cmd_msg);
  serial.add_frame(2, &msense_msg);
  serial.add_frame(3, &rst_msg);

  omni.begin();
}

void loop()
{
  static long last_ctrl = millis();
  static long last_cmd = 0;
  static float speed_x, speed_y, speed_th;
  static double odom_x, odom_y, odom_th;

  serial.update();

  if(serial.read() == 0){
    speed_x = -cmd_msg.data.y * 1000;
    speed_y = cmd_msg.data.x * 1000;
    speed_th = cmd_msg.data.z;
    last_cmd = millis();
    if(rst_msg.data.c == 1){
      omni.move(0, 0, 0);
      M5.Power.reset();
    }
  }else{
    if(last_cmd && (millis()-last_cmd) > timeout_interval){
      speed_x = cmd_msg.data.y = 0;
      speed_y = cmd_msg.data.x = 0;
      speed_th = cmd_msg.data.z = 0;
    }
  }

  omni.get_vel(odom_x, odom_y, odom_th);

  odom_msg.data.x = odom_y / 1000.0;
  odom_msg.data.y = -odom_x / 1000.0;
  odom_msg.data.z = odom_th;

  serial.write(0);

  M5.IMU.getGyroData( &msense_msg.data.gyro.x,
                      &msense_msg.data.gyro.y,
                      &msense_msg.data.gyro.z);
  DEG2RAD(msense_msg.data.gyro.x);
  DEG2RAD(msense_msg.data.gyro.y);
  DEG2RAD(msense_msg.data.gyro.z);
  M5.IMU.getAccelData(&msense_msg.data.acc.x,
                      &msense_msg.data.acc.y,
                      &msense_msg.data.acc.z);
  G2A(msense_msg.data.acc.x);
  G2A(msense_msg.data.acc.y);
  G2A(msense_msg.data.acc.z);
  bmm.read_mag_data();
  msense_msg.data.mag.x = -bmm.mag_data.x;// / 1E6F;
  msense_msg.data.mag.y = bmm.mag_data.y;// / 1E6F;
  msense_msg.data.mag.z = -bmm.mag_data.z;// / 1E6F;

  serial.write(2);

  if(millis() > last_ctrl+ctrl_interval){
    omni.move(speed_x, speed_y, speed_th);
    last_ctrl = millis();
  }

#ifdef DEBUG_MODE
  M5.Lcd.setCursor(0, 0);

  M5.Lcd.printf("vel_tar[mm/s,dec/s]:\nx:%.1f\ny:%.1f\nz:%.1f\n",
                  speed_x, speed_y, speed_th/M_PI*180);
  M5.Lcd.printf("vel_cur[mm/s,dec/s]:\nx:%.1f\ny:%.1f\nz:%.1f\n",
                  speed_x[1], speed_y[1], speed_th[1]/M_PI*180);
  M5.Lcd.printf("vel_fb [mm/s,dec/s]:\nx:%.1lf\ny:%.1lf\nz:%.1lf\n",
                  odom_x, odom_y, odom_th/M_PI*180);
  M5.Lcd.printf("time[ms]:\n%ld\n", millis());

#endif
}
