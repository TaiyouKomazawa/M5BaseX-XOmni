#ifndef WHEEL_VECTOR_H_
#define WHEEL_VECTOR_H_

#include <Arduino.h>

class WheelVector
{
public:
  //pos_x, pos_y, vector_dir[rad]
  WheelVector(double pos_x, double pos_y, double vector_dir, double gain=1.0)
  :  _pos_x(pos_x), _pos_y(pos_y), _v_dir(vector_dir), _gain(gain)
  {
    _pos_r = sqrt(pos_x*pos_x + pos_y*pos_y);

    _SIN = sin(_v_dir);
    _COS = cos(_v_dir);
  }

  virtual inline double set(double x, double y, double th)
  {
    return _gain*(x * _COS + y *_SIN + _pos_r*th);
  }

  virtual inline void get(double &x, double &y, double &th, double v)
  {
    x = _gain * v * _COS; 
    y = _gain * v * _SIN;
    th = _gain * (v / _pos_r);
  }
  
private:
  double _pos_x, _pos_y;
  double _pos_r;
  double _v_dir;
  double _gain;
  double _SIN, _COS;
};

#endif
