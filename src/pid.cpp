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

void PIDController::setDt(double sample_time)
{
    dt = sample_time;
}

double PIDController::compute(double current_temp)
{
    double error = setpoint - current_temp;

    // Proportional
    double p_term = kp * error;

    // Integral + anti-windup
    integral += ki * error * dt;
    integral = clamp(integral, -integral_max, integral_max);

    // Derivative
    double d_term = 0.0;
    if (!first_run)
    {
        d_term = kd * (error - prev_error) / dt;
    }
    first_run = false;

    // Output
    double output = p_term + integral + d_term;
    output = clamp(output, min_output, max_output);

    prev_error = error;
    return output;
}

void PIDController::reset()
{
    integral = 0.0;
    prev_error = 0.0;
    first_run = true;
}