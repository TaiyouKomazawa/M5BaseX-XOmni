#include "WheelOmniEv3.h"


void WheelOmniEv3::begin()
{
  this->reset();
  this->set_pid(0.3, 0.9, 0.001);
  _last_micros = micros();
}

double WheelOmniEv3::set(double x, double y, double th)
{
  double target = WheelVector::set(x, y, th);
  int output = (_forward_step(target) + _pid_step(target)) / 2;
  output = _check_pwm_limit(output);

  _base_x->SetMotorSpeed(_pos, (int8_t)output);
  return (double)output;
}

void WheelOmniEv3::get(double &x, double &y, double &th)
{
  WheelVector::get(x, y, th, _get_mm());
}

void WheelOmniEv3::get_vel(double &x, double &y, double &th)
{
  WheelVector::get(x, y, th, _get_v());
}

void WheelOmniEv3::set_pid(float kp, float ki, float kd)
{
  _kp = kp;
  _ki = ki;
  _kd = kd;
  reset_pid();
}

double WheelOmniEv3::get_wheel_vel()
{
  return _get_v();
}

void WheelOmniEv3::reset()
{
  _base_x->SetMotorSpeed(_pos, 0);
  _base_x->SetEncoderValue(_pos, 0);
  _base_x->SetMode(_pos, NORMAL_MODE);
  _mm[0] = _mm[1] = 0;
  reset_pid();
}

void WheelOmniEv3::reset_pid()
{
  _integral = _prev_v = 0;
}

double WheelOmniEv3::_get_mm()
{
  double deg = _base_x->GetEncoderValue(_pos);
  _mm[0] = _mm[1];
  _mm[1] = M_PI*_wd * (deg / 360.0);
  _interval = (micros() - _last_micros) / 1E6;
  _last_micros = micros();  return _mm[1];
}

double WheelOmniEv3::_get_v()
{
  if(micros() > _last_micros){
    _get_mm();
  }
  return (_mm[1] - _mm[0]) / _interval;
}

int8_t WheelOmniEv3::_pid_step(double target)
{
  double v = _get_v();
  double diff = target - v;

  _integral += diff * _interval;

  double p = diff;
  double d = (v - _prev_v) / _interval;
  _prev_v = v;
  double output = _kp * p + _ki * _integral - _kd * d;
  return WheelOmniEv3::_check_pwm_limit(output);
}

int8_t WheelOmniEv3::_forward_step(double x)
{
  int8_t output = 127* (x / WHEEL_SPEED_LIMIT);
  return _check_pwm_limit(output);
}

inline int8_t WheelOmniEv3::_check_pwm_limit(int x, uint8_t limit)
{
  if (x > limit)
    x = limit;
  else if(x < -limit)
    x = -limit;
  return x;
}
