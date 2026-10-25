#include "control/pid.h"

PID::PID(double p_gain, double i_gain, double d_gain): 
    p_gain_(p_gain), i_gain_(i_gain), d_gain_(d_gain), prev_error_(0.) {}

double PID::calcAccel(double target_velocity){
    double error = target_velocity - curr_velocity_;
    
    double ret = p_gain_ * error;
    if(error <= 5)
        ret += i_gain_ * error * DT;

    ret += d_gain_ * (error - prev_error_) / DT;
    prev_error_ = error;

    // control output clipping -1 ~ 1 (-1 ~ 0 is brake, 0 ~ 1 is accel)
    return std::max(-1.0, std::min(1.0, ret));
}
