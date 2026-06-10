#include "pid.h"

// Low-pass smoothing for the derivative term (0 = none, →1 = heavy). The
// thermocouple reads in 0.25 °C steps, so the raw derivative is very noisy.
static constexpr double D_FILTER = 0.8;

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
      prev_measurement(0.0), d_filtered(0.0), max_output(max_out),
      min_output(min_out), integral_max(integ_max), dt(0.1), first_run(true) {}

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

    // Derivative on the *measurement* (not the error) so a setpoint change
    // doesn't cause a derivative kick, low-pass filtered to tame sensor noise.
    // d(error)/dt = -d(temp)/dt for a fixed setpoint, hence the leading minus.
    double d_term = 0.0;
    if (!first_run && dt > 0.0)
    {
        double deriv = -kd * (current_temp - prev_measurement) / dt;
        d_filtered = D_FILTER * d_filtered + (1.0 - D_FILTER) * deriv;
        d_term = d_filtered;
    }
    first_run = false;
    prev_measurement = current_temp;

    // Output
    output = clamp(p_term + integral + d_term, min_output, max_output);
    return output;
}

void PIDController::reset()
{
    integral = 0.0;
    prev_measurement = 0.0;
    d_filtered = 0.0;
    first_run = true;
}