#pragma once

class PID_controller
{
public:
    double control_signal = 0; // Motor command signal

    double Kp = 1; // proportional gain
    double Ki = 0; // integral gain
    double Kd = 0; // derivative gain
    int T = 50;    // sample time in milliseconds (ms)

    unsigned long last_time = 0;
    bool initialized = false; // becomes true after the first control() call so dt is never computed against a stale/zero last_time
    double total_error = 0, last_error = 0;

    int max_control; // to be preprocessed
    int min_control; // to be preprocessed

    float deadband = 1; // deadband for control signal

    PID_controller(int max_control, int min_control) : max_control(max_control), min_control(min_control) {}

    void control(double sensed_output, double command_value)
    {

        unsigned long current_time = millis(); // returns the number of milliseconds passed since the Arduino started running the program

        if (!initialized)
        {
            last_time = current_time;
            initialized = true;
            return;
        }

        int delta_time = current_time - last_time; // delta time interval

        if (delta_time >= T) // Sufficient time since last loop
        {
            float dt = (float)delta_time / 1000.0; // convert T from milliseconds to seconds
            double error = command_value - sensed_output;

            if (fabs(error) < deadband) // If error is within deadband, set control signal to zero
            {
                control_signal = 0;
                last_error = error; // keep the derivative term consistent for the next out-of-band cycle
                last_time = current_time;
                return;
            }

            // Real error treatment
            double delta_error = error - last_error; // difference of error for derivative term

            double candidate_integral = total_error + error;                                        // provisional integral accumulation
            control_signal = Kp * error + (Ki * dt) * candidate_integral + (Kd / dt) * delta_error; // PID control compute

            // Avoid motor overwork + anti-windup: only keep integrating while the
            // output is not saturated, otherwise total_error runs away.
            if (control_signal >= max_control)
            {
                control_signal = max_control;
            }
            else if (control_signal <= min_control)
            {
                control_signal = min_control;
            }
            else
            {
                total_error = candidate_integral;
            }

            last_error = error;
            last_time = current_time;
        }
    }

    void reset() // Empty all data if motor is deactivated
    {
        total_error = 0;
        last_error = 0;
        control_signal = 0;
        initialized = false;
    }
};