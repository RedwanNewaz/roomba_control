/**
 * Copyright 2021 Redwan Newaz <redwan06me@gmail.com> and
 * Bradley J. Snyder <snyder.bradleyj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "roomba_control/pid.h"

using namespace std;

class PIDImpl
{
    public:
        PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki, double goal_thr );
        ~PIDImpl();
        double calculate( double setpoint, double pv );

    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
        double _goal_thr;

    protected:
        double anti_windup(double u, double e, double thr);
};


PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki, double goal_thr )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki, goal_thr);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID() 
{
    delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki, double goal_thr ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0),
    _goal_thr(goal_thr)
{
}

double PIDImpl::calculate( double setpoint, double pv )
{
    
    // Calculate error
    double error = setpoint - pv;

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

    // Restrict to max/min
    output = anti_windup(output, error, _goal_thr);

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}

double PIDImpl::anti_windup(double u, double e, double thr) {
//    https://www.embeddedrelated.com/showcode/346.php
/* Check for saturation.  In the event of saturation in any one direction,
     inhibit saving the integrator if doing so would deepen the saturation. */
    bool int_ok = true;

    /* Positive saturation? */
    if (u > _max)
    {
        /* Clamp the output */
        u = _max;

        /* Error is the same sign? Inhibit integration. */
        if (e > 0)
        {
            int_ok = false;
        }
    }
        /* Repeat for negative sign */
    else if (u < _min)
    {
        u = _min;

        if (e < 0)
        {
            int_ok = false;
        }
    }

    /* Update the integrator if allowed. */
    if (abs(e) < thr) // my modification: don't integrate error in goal region
    {
        _integral = 0;
    }
    else if (!int_ok)
    {
        _integral -= e * _dt;
    }

    return u;

}

#endif
