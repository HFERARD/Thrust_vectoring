#pragma once
#include "Arduino_LSM9DS1.h"

// #include "Arduino_BMI270_BMM150.h" // Library for the Nano BLE Sense Rev2 IMU

struct Acceleration
{
    float ax = 0, ay = 0, az = 0;
};

struct RotationVelocity
{
    float gx = 0, gy = 0, gz = 0;
};

class GyroscopeAccelerometer
{
public:
    Acceleration acceleration;
    RotationVelocity rotationVelocity;

    void update()
    {
        if (IMU.accelerationAvailable())
        {
            IMU.readAcceleration(acceleration.ax, acceleration.ay, acceleration.az);
        }

        if (IMU.gyroscopeAvailable())
        {
            IMU.readGyroscope(rotationVelocity.gx, rotationVelocity.gy, rotationVelocity.gz);
        }
    }
};

namespace sensor
{
    void setup()
    {
        Serial.begin(115200);
        while (!Serial)
        {
        }

        if (!IMU.begin())
        {
            Serial.println("IMU init failed");
            while (1)
            {
            }
        }
    }
}
