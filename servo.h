#pragma once
#include <Servo.h>
#include "controller.h"

class OpenServoController

{
public:
    const int PIN;
    Servo servo;

    OpenServoController(int pin) : PIN(pin)
    {
        servo.attach(PIN);
    }

    void control(double control_position)
    {
        servo.write(control_position);
    }
};

// class OpenBrushlessController
// {
// private:
//     bool activated = true;
//     int min_throttle = 1000;
//     int max_throttle = 2000;

// public:
//     OpenServoController servo;

//     ServoControllerPID(int pin) : servo(pin)
//     {
//         deactivate();
//     }

//     void activate()
//     {
//         activated = true;
//     }

//     void deactivate()
//     {
//         activated = false;
//         servo.control(90); // Set servo to neutral position
//         pid.reset();
//     }
// }

class ServoControllerPID
{
private:
    bool activated = true;

public:
    OpenServoController servo;
    PID_controller pid;

    ServoControllerPID(int pin, int max_control, int min_control) : servo(pin), pid(max_control, min_control)
    {
        deactivate();
    }

    void control(double command_value, double sensed_output)
    {
        if (!activated) // Check activation status before controlling the servo
            return;
        pid.control(sensed_output, command_value); // Update PID control signal

        servo.control(pid.control_signal + 90);
    }

    void activate()
    {
        activated = true;
    }

    void deactivate()
    {
        activated = false;
        servo.control(90); // Set servo to neutral position
        pid.reset();
    }

    bool get_activated()
    {
        return activated;
    }
};
