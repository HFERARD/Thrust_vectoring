#pragma once

// This file is header to define PID controller
// (No IMU library is needed here -- this is a pure PID class. The IMU library
//  is included once in sensor.h. Including a second/different IMU library here
//  clashes with the global `IMU` object declared by sensor.h.)

class PID_controller
{
public:
    double control_signal = 0; // Motor command signal

    double Kp = 1; // proportional gain
    double Ki = 0; // integral gain
    double Kd = 0; // derivative gain
    int T = 100;   // sample time in milliseconds (ms)

    unsigned long last_time = 0;
    double total_error = 0, last_error = 0;

    int max_control; // to be preprocessed
    int min_control; // to be preprocessed

    PID_controller(int max_control, int min_control) : max_control(max_control), min_control(min_control) {}

    void control(double sensed_output, double command_value)
    {

        unsigned long current_time = millis(); // returns the number of milliseconds passed since the Arduino started running the program

        int delta_time = current_time - last_time; // delta time interval

        if (delta_time >= T) // Sufficient time since last loop
        {

            double error = command_value - sensed_output;

            total_error += error; // accumalates the error

            // Real error treatment
            double delta_error = error - last_error; // difference of error for derivative term

            control_signal = Kp * error + (Ki * T) * total_error + (Kd / T) * delta_error; // PID control compute

            // Avoid motor overwork
            if (control_signal >= max_control)
            {
                Serial.println("control signal overflow");
                control_signal = max_control;
            }
            else if (control_signal <= min_control)
            {
                control_signal = min_control;
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
    }
};