#pragma once
#include <IBusBM.h>
#include <tuple>

class RadioController
{
private:
    IBusBM IBus;

    const unsigned long T = 10;
    unsigned long last_time = 0;

    int rcRoll = 1500;
    int rcPitch = 1500;
    int rcThrottle = 1000;

    float rollCommand = 0;
    float pitchCommand = 0;
    float throttleCommand = 0; // Percentage throttle command

    float maxPitch = 20.0; // Max value on control levers
    float maxRoll = 20.0;  // Max value on control levers

public:
    void begin()
    {
        IBus.begin(Serial1);
        Serial.println("Radio controller initialized.");
    }

    std::tuple<float, float, float> getCommand()
    {
        unsigned long current_time = millis();

        int delta_time = current_time - last_time;

        if (delta_time >= T)
        { // Sufficient time since last loop

            last_time = current_time;

            rcThrottle = IBus.readChannel(2);
            rcPitch = IBus.readChannel(1);
            rcRoll = IBus.readChannel(0);

            rollCommand = constrain((rcRoll - 1500) * (maxRoll / 500.0), -maxRoll, maxRoll);
            pitchCommand = constrain((rcPitch - 1500) * (maxPitch / 500.0), -maxPitch, maxPitch);
            throttleCommand = constrain((rcThrottle - 1000) / 10.0, 0.0, 100.0);

            // Diagnostic: cnt_rec is the number of valid iBus frames the library
            // has actually decoded. If it stays 0 the receiver is NOT reaching
            // Serial1/D0 (wiring or receiver not in iBus mode) -- not a code bug.
            // If it climbs but the raw channels stay at 0, the frames are being
            // read but the channel mapping/receiver output is off.
            Serial.print("RC frames=");
            Serial.print(IBus.cnt_rec);
            Serial.print(" raw ch0/1/2=");
            Serial.print(rcRoll);
            Serial.print('/');
            Serial.print(rcPitch);
            Serial.print('/');
            Serial.println(rcThrottle);
        }

        return std::make_tuple(rollCommand, pitchCommand, throttleCommand);
    }
};
