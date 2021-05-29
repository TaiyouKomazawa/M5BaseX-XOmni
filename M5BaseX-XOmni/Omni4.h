#ifndef OMNI_4_H_
#define OMNI_4_H_

#include "WheelOmniEv3.h"

class Omni4{
public:
  Omni4(WheelOmniEv3 &LF, WheelOmniEv3 &LB, WheelOmniEv3 &RB, WheelOmniEv3 &RF)
  : _LF(&LF), _LB(&LB), _RB(&RB), _RF(&RF)
  {}

  void begin()
  {
    _LF->begin();
    _LB->begin();
    _RB->begin();
    _RF->begin();
  }

  void get_odom(double &x, double &y, double &th)
  {
    double lf_x, lf_y, lf_th;
    double lb_x, lb_y, lb_th;
    double rb_x, rb_y, rb_th;
    double rf_x, rf_y, rf_th;
    
    _LF->get(lf_x, lf_y, lf_th);
    _LB->get(lb_x, lb_y, lb_th);
    _RB->get(rb_x, rb_y, rb_th);
    _RF->get(rf_x, rf_y, rf_th);

    x = (lf_x+lb_x+rb_x+rf_x)/4.0;
    y = (lf_y+lb_y+rb_y+rf_y)/4.0;
    th = (lf_th+lb_th+rb_th+rf_th)/4.0;
  }

  void get_vel(double &x, double &y, double &th)
  {
    double lf_x, lf_y, lf_th;
    double lb_x, lb_y, lb_th;
    double rb_x, rb_y, rb_th;
    double rf_x, rf_y, rf_th;
    
    _LF->get_vel(lf_x, lf_y, lf_th);
    _LB->get_vel(lb_x, lb_y, lb_th);
    _RB->get_vel(rb_x, rb_y, rb_th);
    _RF->get_vel(rf_x, rf_y, rf_th);

    x = (lf_x+lb_x+rb_x+rf_x)/4.0;
    y = (lf_y+lb_y+rb_y+rf_y)/4.0;
    th = (lf_th+lb_th+rb_th+rf_th)/4.0;
  }
private:
  WheelOmniEv3 *_LF, *_LB, *_RB, *_RF;
};

#endif
