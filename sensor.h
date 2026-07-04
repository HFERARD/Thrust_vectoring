#pragma once
// #include "Arduino_LSM9DS1.h"

#include "Arduino_BMI270_BMM150.h" // Library for the Nano BLE Sense Rev2 IMU

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

    float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

    // Average accelerometer reading captured during the still-hold calibration.
    // Used to establish the level reference for the attitude estimate -- the
    // gyro bias alone cannot remove a constant pitch/roll offset (that offset
    // comes from the accelerometer, not the gyro).
    float accRestX = 0, accRestY = 0, accRestZ = 1;

    void begin()
    {
        // The IMU driver must be initialized before any read. Without this the
        // internal I2C bus (Wire1 on the Nano 33 BLE) is never configured, and
        // touching it hard-faults the MCU -- which, on a native-USB board, tears
        // down the USB CDC port ("board disconnects right after upload").
        if (!IMU.begin())
        {
            Serial.println("IMU init failed");
            while (1)
            {
                // Blink so the fault is visible instead of hanging silently.
                digitalWrite(LED_BUILTIN, HIGH);
                delay(100);
                digitalWrite(LED_BUILTIN, LOW);
                delay(100);
            }
        }

        const int SAMPLES = 2000;

        float sumX = 0, sumY = 0, sumZ = 0;
        float accSumX = 0, accSumY = 0, accSumZ = 0;
        float x, t, z;
        float ax, ay, az;
        int count = 0;
        int accCount = 0;

        Serial.println("Calibrating gyroscope... Please keep the device still and level.");

        // Bound the calibration so a silent IMU can never hang setup forever.
        unsigned long calStart = millis();
        while (count < SAMPLES && millis() - calStart < 15000)
        {
            if (IMU.gyroscopeAvailable())
            {
                IMU.readGyroscope(x, t, z);
                sumX += x;
                sumY += t;
                sumZ += z;
                count++;
                delay(5);
            }
            // Average the accelerometer over the same still period so we can
            // zero the attitude estimate to this level reference.
            if (IMU.accelerationAvailable())
            {
                IMU.readAcceleration(ax, ay, az);
                accSumX += ax;
                accSumY += ay;
                accSumZ += az;
                accCount++;
            }
        }
        if (count > 0)
        {
            gyroBiasX = sumX / count;
            gyroBiasY = sumY / count;
            gyroBiasZ = sumZ / count;
        }

        if (accCount > 0)
        {
            accRestX = accSumX / accCount;
            accRestY = accSumY / accCount;
            accRestZ = accSumZ / accCount;
        }

        Serial.println("Gyroscope calibration complete.");
    }

    void update()
    {
        if (IMU.accelerationAvailable())
        {
            IMU.readAcceleration(acceleration.ax, acceleration.ay, acceleration.az);
        }

        if (IMU.gyroscopeAvailable())
        {
            IMU.readGyroscope(rotationVelocity.gx, rotationVelocity.gy, rotationVelocity.gz);
            rotationVelocity.gx -= gyroBiasX;
            rotationVelocity.gy -= gyroBiasY;
            rotationVelocity.gz -= gyroBiasZ;
        }
    }
};

namespace sensor
{
    void setup()
    {
        Serial.begin(115200);
        unsigned long t0 = millis();
        while (!Serial && millis() - t0 < 2000)
        {
            // Wait up to 2s for a serial monitor, then boot anyway so the
            // board still runs when it isn't tethered to a computer.
        }

        if (!IMU.begin())
        {
            Serial.println("IMU init failed");
            while (1)
            {
                // Blink instead of hanging silently so the fault is visible.
                digitalWrite(LED_BUILTIN, HIGH);
                delay(100);
                digitalWrite(LED_BUILTIN, LOW);
                delay(100);
            }
        }

        // Boot banner: if this line reappears in the serial console while the
        // board is supposedly just running, the board is resetting (brown-out
        // or crash), not streaming normally.
        Serial.println("=== BOOT ===");
    }
}
