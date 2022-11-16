#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;

class PIDImpl
{
    public:
        PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
        ~PIDImpl();
        double calculate( double error);
        void reset();
        void change_P_gain(float K);
    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
        double _pre_integral;
};


PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint - pv);
}
double PID::calculate2( double err )
{
    return pimpl->calculate(err);
}
void PID::reset()
{
    pimpl->reset();
}
void PID::change_P(float K_p){
    pimpl->change_P_gain(K_p);
}

PID::~PID() 
{
    delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0),
    _pre_integral(0)
{
}

double PIDImpl::calculate( double error)
{
    
    // Calculate error
//    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;
    // double output;
    // if (_Ki == 0)
    // output = Pout + Iout + Dout;
    // else
    // output  = _pre_error + Pout;

    // Restrict to max/min
    if( output > _max ){
        _integral = _pre_integral;
        output = _max;
    }
    else if( output < _min ){
        _integral = _pre_integral;
        output = _min;
    }
    _pre_integral = _integral;

    // Save error to previous error
    _pre_error = error;

    return output;
}

void PIDImpl::reset(){
    _pre_error = 0;
    _integral = 0;
}

void PIDImpl::change_P_gain(float K){
    _Kp = K;
}

PIDImpl::~PIDImpl()
{
}
