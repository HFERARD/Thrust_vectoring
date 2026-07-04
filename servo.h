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
    }

    void begin()
    {
        servo.attach(PIN); // Attach servo to pin
    }

    void control(double control_position)
    {
        servo.write(control_position);
    }
};

// This code should be updated, there is shared code with servo controllers that should be joined

class OpenBrushlessController
{
private:
    int PIN;
    bool activated = false;
    int min_throttle = 1000;
    int max_throttle = 2000;
    float throttle = 0;

public:
    Servo esc; // Electronic Speed Controller (ESC) for brushless motor

    OpenBrushlessController(int pin) : PIN(pin)
    {
    }

    void begin()
    {
        esc.attach(PIN, min_throttle, max_throttle);
        deactivate();
        delay(3000);
    }

    void activate()
    {
        activated = true;
    }

    void deactivate()
    {
        setThrottle(0); // Set throttle to 0%
        activated = false;
    }

    void control()
    {
        if (!activated) // Check activation status before controlling the ESC
            return;
        esc.writeMicroseconds(min_throttle + (max_throttle - min_throttle) * throttle);
    }

    void control(float percentage)
    {
        if (!activated) // Check activation status before controlling the ESC
            return;
        setThrottle(percentage);
        esc.writeMicroseconds(min_throttle + (max_throttle - min_throttle) * throttle);
    }

    void setThrottle(float percentage)
    {
        if (!activated) // Check activation status before setting throttle
            return;
        throttle = percentage / 100.0f; // Convert to 0.0 - 1.0 range
    }

    float getThrottle()
    {
        if (!activated) // Check activation status before getting throttle
            return 0;
        return throttle * 100.0f; // Convert to 0.0 - 100.0 range
    }
};

class ServoControllerPID
{
private:
    bool activated = false;

public:
    OpenServoController servo;
    PID_controller pid;

    ServoControllerPID(int pin, int max_control, int min_control) : servo(pin), pid(max_control, min_control)
    {
    }

    void begin()
    {
        servo.begin();
        deactivate(); // Set servo to neutral position and reset PID
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
