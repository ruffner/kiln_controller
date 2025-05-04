#ifndef TEMPERATURE_CONTROLLER_H
#define TEMPERATURE_CONTROLLER_H

#include <PID_v1.h>

class TemperatureController {
public:
    // Constructor with predefined PID constants
    TemperatureController()
        : input(0), output(0), setpoint(0),
          pid(&input, &output, &setpoint, 20.0, 0.2, 0.1, DIRECT) {
        pid.SetMode(AUTOMATIC);
        pid.SetOutputLimits(0, 1);  // Output duty cycle between 0.0 and 1.0
    }

    // Set the target temperature
    void setSetpoint(double newSetpoint) {
        setpoint = newSetpoint;
    }

    // Optionally set output limits (0-1 is default)
    void setOutputLimits(double min, double max) {
        pid.SetOutputLimits(min, max);
    }

    // Update the controller with current temperature and compute new output
    double update(double currentTemperature) {
        input = currentTemperature;
        pid.Compute();
        return output;
    }

    // Getters
    double getSetpoint() const { return setpoint; }
    double getOutput() const { return output; }

private:
    double input;     // Current temperature
    double output;    // Output duty cycle (0.0 - 1.0)
    double setpoint;  // Target temperature
    PID pid;
};

#endif // TEMPERATURE_CONTROLLER_H
