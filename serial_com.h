#pragma once
#include "sensor.h"
#include "servo.h"

using namespace std;

class SerialHandler
{
    int T = 200; // Sample time to send data to computer
    unsigned long last_time = 0;

    bool GYR = true; // Gyroscopic data
    bool ACC = true; // Accelerometric data
    bool ROL = true; // Estimated roll angle
    bool SRO = true; // Servo roll command
    bool PIT = true; // Estimated pitch angle
    bool SPI = true; // Servo pitch command

private:
    // This class handles all outgoing communication to the nano board
    GyroscopeAccelerometer &imu;
    ServoControllerPID &servoRoll;  // index 1
    ServoControllerPID &servoPitch; // index 2
    // ServoControllerPID &brushlessPID; // index 3

    int digitsACCprecision = 5;
    int digitsGYRprecision = 5;
    int digitsROLprecision = 5;
    int digitsPITprecision = 5;
    int digitsSROprecision = 5;
    int digitsSPIprecision = 5;

public:
    SerialHandler(GyroscopeAccelerometer &imu,
                  ServoControllerPID &servoRoll,
                  ServoControllerPID &servoPitch)
        : imu(imu), servoRoll(servoRoll), servoPitch(servoPitch)
    {
    }

    void updateReader()
    {
        if (!Serial.available()) // Check whether data is stored in the reading buffer
            return;

        String line = Serial.readStringUntil('\n'); // Get first line
        line.trim();
        if (line.length() < 2)
        { // If line is single character or empty, ignore it
            Serial.println("Invalid command");
            return;
        }

        // Command format : cmd <params>
        int sep = line.indexOf(' ');
        String cmd = line.substring(0, sep);
        String params = line.substring(sep + 1);
        int sep2 = params.indexOf(' ');

        int motor;
        float val;

        if (sep2 != -1)
        {
            motor = params.substring(0, sep2).toInt();
            val = params.substring(sep2 + 1).toFloat();
        }
        else
        {
            motor = params.toInt();
            val = 0;
        }

        ServoControllerPID *ptr = nullptr;

        switch (motor)
        {
        case 1:
            ptr = &servoRoll;
            break;
        case 2:
            ptr = &servoPitch;
            break;
            // case 3:
            //  ptr = &brushlessPID;
            //    break;

        default:
            Serial.print("unknown motor index: ");
            Serial.println(motor);
            return;
        }

        // Check for each available command
        switch (cmd[0])
        {
        case 'p': // Kp
            ptr->pid.Kp = val;
            break;
        case 'i': // Ki
            ptr->pid.Ki = val;
            break;
        case 'd': // Kd
            ptr->pid.Kd = val;
            break;
        // case 's': // Change value of command setter
        //     ptr->pid.command_value = val;
        //     break;
        case 't': // Time step in milliseconds
            ptr->pid.T = (int)val;
            break;
        case '+': // Activate all
            ptr->activate();
            break;
        case '-': // Deactivate all
            ptr->deactivate();
            break;
        case '?': // Print all parameters
            Serial.print("Kp=");
            Serial.print(ptr->pid.Kp);
            Serial.print(" Ki=");
            Serial.print(ptr->pid.Ki);
            Serial.print(" Kd=");
            Serial.print(ptr->pid.Kd);
            // Serial.print(" setpoint=");
            // Serial.print(ptr->pid.command_value);
            Serial.print(" T=");
            Serial.println(ptr->pid.T);
            break;
        default:
            Serial.print("unknown cmd: ");
            Serial.println(cmd);
            return;
        }
    }

    void sendData(float roll, float pitch, float rollCommand, float pitchCommand, Acceleration &acc, RotationVelocity &gyr)
    {
        unsigned long current_time = millis(); // returns the number of milliseconds passed since the Arduino started running the program

        int delta_time = current_time - last_time; // delta time interval

        if (delta_time < T) // Sufficient time since last loop
            return;

        last_time = current_time;

        int margin = 0;
        int bytes = Serial.availableForWrite();
        if (bytes < margin) // A large margin [COULD BE BETTER] to avoid buffer overload
        {
            return;
        }

        if (GYR)
        {
            Serial.print("GYR ");
            Serial.print(gyr.gx, digitsGYRprecision);
            Serial.print(',');
            Serial.print(gyr.gy, digitsGYRprecision);
            Serial.print(',');
            Serial.println(gyr.gz, digitsGYRprecision);
        }

        if (ACC)
        {
            Serial.print("ACC ");
            Serial.print(acc.ax, digitsACCprecision);
            Serial.print(',');
            Serial.print(acc.ay, digitsACCprecision);
            Serial.print(',');
            Serial.println(acc.az, digitsACCprecision);
        }

        if (ROL)
        {
            Serial.print("ROL ");
            Serial.println(roll, digitsROLprecision);
        }

        if (PIT)
        {
            Serial.print("PIT ");
            Serial.println(pitch, digitsPITprecision);
        }

        if (SRO)
        {
            Serial.print("SRO ");
            Serial.println(rollCommand, digitsSROprecision);
        }

        if (SPI)
        {
            Serial.print("SPI ");
            Serial.println(pitchCommand, digitsSPIprecision);
        }
    }
};
