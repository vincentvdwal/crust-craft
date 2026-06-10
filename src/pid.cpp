#include "pid.h"

double PIDController::clamp(double value, double min_val, double max_val) const
{
    if (value < min_val)
        return min_val;
    if (value > max_val)
        return max_val;
    return value;
}

PIDController::PIDController(double Kp, double Ki, double Kd,
                             double max_out, double min_out, double integ_max)
    : kp(Kp), ki(Ki), kd(Kd), setpoint(0.0), integral(0.0),
      prev_error(0.0), max_output(max_out), min_output(min_out),
      integral_max(integ_max), dt(0.1), first_run(true) {}

void PIDController::setSetpoint(double sp)
{
    setpoint = sp;
}

void PIDController::setSetKp(double nkp)
{
    kp = nkp;
}

void PIDController::setSetKi(double nki)
{
    ki = nki;
}

void PIDController::setSetKd(double nkd)
{
    kd = nkd;
}

void PIDController::setDt(double sample_time)
{
    dt = sample_time;
}

double PIDController::compute(double current_temp)
{
    double error = setpoint - current_temp;

    // Proportional
    double p_term = kp * error;

    // Tentative output using the integral as it currently stands.
    double output = p_term + integral;

    // Integral with conditional-integration anti-windup.
    // Only accumulate when we are NOT already saturated in the direction the
    // error would push us further. During a long heat-up the output sits at
    // 100% for minutes; without this guard the integral winds up enormously
    // and causes a large overshoot when the setpoint is finally reached.
    bool saturated_high = output >= max_output && error > 0.0;
    bool saturated_low = output <= min_output && error < 0.0;
    if (!saturated_high && !saturated_low)
    {
        integral += ki * error * dt;
        integral = clamp(integral, -integral_max, integral_max);
    }

    // Output
    output = clamp(p_term + integral, min_output, max_output);

    prev_error = error;
    return output;
}

void PIDController::reset()
{
    integral = 0.0;
    prev_error = 0.0;
    first_run = true;
}