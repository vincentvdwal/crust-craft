#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController
{
private:
    double kp, ki, kd, setpoint, integral, prev_error;
    double max_output, min_output, integral_max, dt;
    bool first_run;

    // Clamp function (C++11 compatible)
    double clamp(double value, double min_val, double max_val) const;

public:
    PIDController(double Kp, double Ki, double Kd,
                  double max_out = 100.0, double min_out = 0.0,
                  double integ_max = 50.0);

    void setSetpoint(double sp);
    void setSetKp(double nkp);
    void setSetKi(double nki);
    void setSetKd(double nkd);

    void setDt(double sample_time);
    double compute(double current_temp);
    void reset();
};

#endif