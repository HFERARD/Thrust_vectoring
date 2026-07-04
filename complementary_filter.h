#pragma once
#include <cmath>
#include "sensor.h"

// DOCUMENT inspired from espp-cpp library

namespace espp
{
    /// Complementary filter for estimating pitch and roll angles from
    /// accelerometer and gyroscope readings.
    /// The filter is defined by the following equation:
    ///  angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel
    ///  where:
    ///  - angle is the estimated angle,
    ///  - alpha is the filter coefficient (0 < alpha < 1),
    ///  - gyro is the gyroscope reading,
    ///  - accel is the accelerometer reading,
    ///  - dt is the time step, in seconds.
    class ComplementaryFilter
    {
    private:
        float alpha;
        float pitch = 0.0f, roll = 0.0f;

        // Level-reference offsets. The estimated angle converges to the
        // accelerometer angle at rest, which is non-zero whenever the IMU is
        // not perfectly level (mounting tilt / zero-g bias). These offsets are
        // captured once at a known-level position and subtracted on output so
        // "flat" reads 0. NOTE: gyro-bias calibration does NOT remove this --
        // it only affects the integrated gyro term, not the steady-state angle.
        float pitchOffset = 0.0f, rollOffset = 0.0f;

        unsigned long last_time = 0;
        bool initialized = false;

    public:
        explicit ComplementaryFilter(float alpha = 0.5f)
            : alpha(alpha) {}

        void update(Acceleration &acc, RotationVelocity &gyr)
        {
            unsigned long current_time = millis();

            if (!initialized)
            {
                // First call: no valid previous timestamp yet
                last_time = current_time;
                initialized = true;
                return;
            }

            // millis() returns ms -- convert the delta to seconds so it
            float dt = (current_time - last_time) / 1000.0f;
            last_time = current_time;

            update(acc.ax, acc.ay, acc.az, gyr.gx, gyr.gy, gyr.gz, dt);
        }

        void update(float ax, float ay, float az, float gx, float gy, float gz, float dt)
        {

            float accelPitch = atan2f(ay, sqrtf(ax * ax + az * az)) * 180.0f / (float)M_PI;
            float accelRoll = atan2f(-ax, az) * 180.0f / (float)M_PI;

            float gyroPitch = pitch + gx * dt;
            float gyroRoll = roll + gy * dt;

            // Apply complementary filter
            pitch = alpha * gyroPitch + (1 - alpha) * accelPitch;
            roll = alpha * gyroRoll + (1 - alpha) * accelRoll;
        }

        void zeroFromAccel(float ax, float ay, float az)
        {
            pitchOffset = atan2f(ay, sqrtf(ax * ax + az * az)) * 180.0f / (float)M_PI;
            rollOffset = atan2f(-ax, az) * 180.0f / (float)M_PI;
        }

        float get_pitch() const { return pitch - pitchOffset; }

        float get_roll() const { return roll - rollOffset; }
    };
}