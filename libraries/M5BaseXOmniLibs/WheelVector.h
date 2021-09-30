#ifndef WHEEL_VECTOR_H_
#define WHEEL_VECTOR_H_

#include <Arduino.h>

class WheelVector
{
public:
  WheelVector(double wheel_dist, double vector_dir, double gain = 1.0)
      : _round(2*wheel_dist*M_PI), _v_dir(vector_dir), _gain(gain)
  {
    _SIN = sin(_v_dir);
    _COS = cos(_v_dir);
  }

  virtual double set(double x, double y, double th)
  {
    double r_per_th = _round * th / (2*M_PI);
    return _gain * (x * _COS + y * _SIN + r_per_th);
  }

  virtual void get(double &x, double &y, double &th, double v)
  {
    x = _gain * v / _COS;
    y = _gain * v / _SIN;
    th = _gain * 2 * M_PI * v / _round;
  }

private:
  double _round;
  double _pos_r;
  double _v_dir;
  double _gain;
  double _SIN, _COS;
};

#endif
