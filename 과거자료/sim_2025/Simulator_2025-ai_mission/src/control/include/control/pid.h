#include <ros/ros.h>

constexpr double DT = 0.02;

class PID {
public:
    PID(double p_gain, double i_gain, double d_gain);
    double calcAccel(double target_target_velocity);
    
    void setCurrVelocity(double velocity){
        curr_velocity_ = velocity;
    }

private:
    const double p_gain_, i_gain_, d_gain_;
    double curr_velocity_;
    double prev_error_;
};
