#ifndef WHEEL_OMNI_EV3_H_
#define WHEEL_OMNI_EV3_H_

#include "BaseX.h"
#include "WheelVector.h"

#define WHEEL_SPEED_LIMIT 465.0 //[mm/s]

class WheelOmniEv3 : public WheelVector 
{
public:
  WheelOmniEv3(double pos_x_mm, double pos_y_mm, double vector_dir, double wheel_diameter, uint8_t pos, BASE_X &base_x)
  : WheelVector(pos_x_mm, pos_y_mm, vector_dir, -1.0), _wd(wheel_diameter),
    _pos(pos), _base_x(&base_x)
  {}

  void begin()
  {
    this->reset();
    this->set_pid(0.3, 0.9, 0);
    _last_micros = micros();
  }

  virtual inline double set(double x, double y, double th)
  { 
    double target = WheelVector::set(x, y, th);
    int output = _forward_step(target) + _pid_step(target);
    output = _check_pwm_limit(output);
    
    _base_x->SetMotorSpeed(_pos, output);
    return output;
  }

  virtual inline void get(double &x, double &y, double &th)
  {
    WheelVector::get(x, y, th, _get_mm());
  }

  virtual inline void get_vel(double &x, double &y, double &th)
  {
    WheelVector::get(x, y, th, _get_v());
  }

  void set_pid(float kp, float ki, float kd)
  {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    reset_pid();
  }

  double get_wheel_vel()
  {
    return _get_v();
  }

  void reset()
  {
    _base_x->SetMotorSpeed(_pos, 0);
    _base_x->SetEncoderValue(_pos, 0);
    _base_x->SetMode(_pos, NORMAL_MODE);
    _mm[0] = _mm[1] = 0;
    reset_pid();
  }
  
  void reset_pid()
  {
    _integral = _prev_v = 0;
  }
private:
  double _get_mm()
  {
    double deg = _base_x->GetEncoderValue(_pos);
    _mm[0] = _mm[1];
    _mm[1] = M_PI*_wd * (deg / 360.0);
    _interval = (micros() - _last_micros) / 1E6;
    _last_micros = micros();

    return _mm[1];
  }

  double _get_v()
  {
    if(micros() > _last_micros){
      _get_mm();
    }
    return (_mm[1] - _mm[0]) / _interval;
  }
  
  int8_t _pid_step(double target)
  {
    double v = _get_v();
    double diff = target - v;
    
    _integral += diff * _interval;
    
    double p = diff;
    double d = (v - _prev_v) / _interval;
    _prev_v = v;
    double output = _kp * p + _ki * _integral - _kd * d;

    return _check_pwm_limit(output);
  }

  int8_t _forward_step(double x)
  {
    int8_t output = 127* (x / WHEEL_SPEED_LIMIT);    
    return _check_pwm_limit(output);
  }

  inline int8_t _check_pwm_limit(int x)
  {
    if(x > 127)
      x = 127;
    else if(x < -127)
      x = -127;
    return x;
  }

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
