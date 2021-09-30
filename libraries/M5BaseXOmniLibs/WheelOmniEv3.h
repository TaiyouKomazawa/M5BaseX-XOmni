#ifndef WHEEL_OMNI_EV3_H_
#define WHEEL_OMNI_EV3_H_

#include <BaseX.h>
#include <WheelVector.h>

#define WHEEL_SPEED_LIMIT 465.0 //[mm/s]

class WheelOmniEv3 : public WheelVector
{
public:
  WheelOmniEv3(double wheel_dist, double vector_dir, double wheel_diameter, uint8_t pos, BASE_X &base_x)
      : WheelVector(wheel_dist, vector_dir, -1.0), _wd(wheel_diameter),
        _pos(pos), _base_x(&base_x)
  {}

  void begin();

  virtual double set(double x, double y, double th);
  virtual void get(double &x, double &y, double &th);
  virtual void get_vel(double &x, double &y, double &th);

  void set_pid(float kp, float ki, float kd);

  double get_wheel_vel();

  void reset();
  void reset_pid();
private:
  double _get_mm();
  double _get_v();

  int8_t _pid_step(double target);

  int8_t _forward_step(double x);

  inline int8_t _check_pwm_limit(int x, uint8_t limit = 127);

  double _wd;

  long _last_micros;
  double _mm[2];
  double _interval;

  float _kp, _ki, _kd;
  double _integral, _prev_v;

  uint8_t _pos;
  BASE_X *_base_x;
};

#endif
