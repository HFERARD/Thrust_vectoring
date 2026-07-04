#pragma once
#include <ArduinoBLE.h>
#include "sensor.h"
#include "servo.h"

// THIS IS AN EXPERIMENTAL BLE IMPLEMENTATION
// It is partly implemented with ai and not fully tested
// Not to be used in production code

using namespace std;

#define BLE_UART_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_UART_RX_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_UART_TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class HandlerBLE
{
    int T = 200; // Sample time to send data to computer
    unsigned long last_time = 0;

    bool GYR = true; // Gyroscopic data
    bool ACC = true; // Accelerometric data
    bool ROL = true; // Estimated roll angle
    bool SRO = true; // Servo roll command
    bool PIT = true; // Estimated pitch angle
    bool SPI = true; // Servo pitch command
    bool THR = true; // Brushless throttle command

private:
    // This class handles all communication with the computer over BLE
    GyroscopeAccelerometer &imu;

    ServoControllerPID &servoRoll;      // index 1
    ServoControllerPID &servoPitch;     // index 2
    OpenBrushlessController &brushless; // index 3

    int digitsACCprecision = 5;
    int digitsGYRprecision = 5;
    int digitsROLprecision = 5;
    int digitsPITprecision = 5;
    int digitsSROprecision = 5;
    int digitsSPIprecision = 5;
    int digitsTHRprecision = 5;

    // BLE plumbing. The characteristics hold up to 128 bytes; outgoing payloads
    // are still split into small chunks (see writeChunked) so delivery does not
    // depend on the central negotiating a large MTU.
    const char *deviceName;
    BLEService uartService;
    BLECharacteristic rxChar;
    BLECharacteristic txChar;

    String rxBuffer; // accumulates inbound bytes until a full '\n' line
    bool wasConnected = false;

    // ---- outbound helpers ---------------------------------------------------

    // Split an arbitrary payload into <=20-byte notifications. 20 bytes is the
    // usable payload of the default 23-byte BLE MTU, so this works even if the
    // central never enlarges the MTU. Newlines are preserved inside the byte
    // stream, so the receiver reassembles lines exactly like a serial port.
    void writeChunked(const String &s)
    {
        if (!txChar.subscribed()) // nobody is listening; drop it like an idle serial port
            return;

        const int maxChunk = 20;
        int len = s.length();
        int i = 0;
        while (i < len)
        {
            int n = min(maxChunk, len - i);

            // writeValue() returns false when the notification queue is full.
            // If we ignore that, the chunk -- and the '\n' inside it -- is lost,
            // and the PC sees two telemetry lines fused into one unparsable line
            // (e.g. "THR 0.0GYR 1,2,3"). Poll + retry until the chunk is actually
            // queued so the outgoing byte stream stays intact.
            int tries = 0;
            while (!txChar.writeValue((const uint8_t *)(s.c_str() + i), n))
            {
                BLE.poll();
                if (!txChar.subscribed() || ++tries > 100)
                    return; // central went away mid-frame; drop the rest
            }

            i += n;
            BLE.poll(); // let the stack flush the notification before the next chunk
        }
    }

    // Equivalent of Serial.println(...) for command replies.
    void sendLine(const String &s)
    {
        writeChunked(s + "\n");
    }

    // Restart advertising after a central disconnects so the board stays
    // discoverable, mirroring how a USB serial port is always ready to reconnect.
    void manageConnection()
    {
        bool now = BLE.connected();
        if (!now && wasConnected)
            BLE.advertise();
        wasConnected = now;
    }

    // ---- inbound command parsing -------------------------------------------
    // Identical semantics to SerialHandler::updateReader, just operating on a
    // line handed in from the BLE RX buffer and replying over BLE.
    void processLine(String line)
    {
        line.trim();
        if (line.length() < 2)
        { // If line is single character or empty, ignore it
            sendLine("Invalid command");
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

        case 3:
            if (cmd[0] == 'v') // Brushless controller does not have PID parameters to set
            {
                brushless.setThrottle(val);
                return;
            }
            else if (cmd[0] == '+' || cmd[0] == '-') // Activate or deactivate brushless controller
            {
                if (cmd[0] == '+')
                    brushless.activate();
                else
                    brushless.deactivate();
                return;
            }
            else
            {
                sendLine("unknown cmd for brushless: " + cmd);
                return;
            }
            break;
        default:
            sendLine("unknown motor index: " + String(motor));
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
        {
            String reply = "Kp=";
            reply += String(ptr->pid.Kp);
            reply += " Ki=";
            reply += String(ptr->pid.Ki);
            reply += " Kd=";
            reply += String(ptr->pid.Kd);
            reply += " T=";
            reply += String(ptr->pid.T);
            sendLine(reply);
            break;
        }
        default:
            sendLine("unknown cmd: " + cmd);
            return;
        }
    }

public:
    HandlerBLE(GyroscopeAccelerometer &imu,
               ServoControllerPID &servoRoll,
               ServoControllerPID &servoPitch,
               OpenBrushlessController &brushless,
               const char *deviceName = "MotorControl")
        : imu(imu), servoRoll(servoRoll), servoPitch(servoPitch), brushless(brushless),
          deviceName(deviceName),
          uartService(BLE_UART_SERVICE_UUID),
          rxChar(BLE_UART_RX_UUID, BLEWrite | BLEWriteWithoutResponse, 128),
          txChar(BLE_UART_TX_UUID, BLERead | BLENotify, 128)
    {
    }

    // Bring up the radio and start advertising the UART service. Call once in
    // setup(), the same way Serial.begin() is called for the wired handler.
    bool begin()
    {
        if (!BLE.begin())
        {
            Serial.println("BLE.begin() failed");
            return false;
        }

        BLE.setLocalName(deviceName);
        BLE.setDeviceName(deviceName);

        uartService.addCharacteristic(rxChar);
        uartService.addCharacteristic(txChar);
        BLE.addService(uartService);

        // NB: do NOT advertise the 128-bit service UUID here.
        //   flags(3) + local name "MotorControl"(14) + 128-bit UUID(18) = 35 bytes,
        // which overflows the 31-byte BLE advertising packet. When that happens
        // BLE.advertise() fails and the board never becomes discoverable. The PC
        // scans/matches by local name instead, so the advertised UUID isn't needed.
        //   BLE.setAdvertisedService(uartService);   // <- leave disabled

        if (!BLE.advertise())
        {
            Serial.println("BLE.advertise() failed");
            return false;
        }

        Serial.print("BLE advertising as '");
        Serial.print(deviceName);
        Serial.print("'  MAC=");
        Serial.println(BLE.address());
        return true;
    }

    void updateReader()
    {
        BLE.poll();         // service the BLE stack (connections, writes, notifications)
        manageConnection(); // re-advertise after a disconnect

        // Drain whatever the central wrote into our line buffer.
        if (rxChar.written())
        {
            int len = rxChar.valueLength();
            const uint8_t *data = rxChar.value();
            for (int i = 0; i < len; i++)
                rxBuffer += (char)data[i];
        }

        // Process every complete '\n'-terminated line, keeping any partial tail.
        int nl;
        while ((nl = rxBuffer.indexOf('\n')) != -1)
        {
            String line = rxBuffer.substring(0, nl);
            rxBuffer = rxBuffer.substring(nl + 1);
            processLine(line);
        }
    }

    void sendData(float roll, float pitch, float rollCommand, float pitchCommand, Acceleration &acc, RotationVelocity &gyr, float throttleCommand)
    {
        unsigned long current_time = millis(); // number of milliseconds since the program started

        int delta_time = current_time - last_time; // delta time interval

        if (delta_time < T) // Sufficient time since last loop
            return;

        last_time = current_time;

        if (!txChar.subscribed()) // no BLE client subscribed; nothing to stream to
            return;

        // Build the whole telemetry frame as one string, then push it out in
        // small BLE notifications. Same line format as the serial handler.
        String out;

        if (GYR)
        {
            out += "GYR ";
            out += String(gyr.gx, digitsGYRprecision);
            out += ',';
            out += String(gyr.gy, digitsGYRprecision);
            out += ',';
            out += String(gyr.gz, digitsGYRprecision);
            out += '\n';
        }

        if (ACC)
        {
            out += "ACC ";
            out += String(acc.ax, digitsACCprecision);
            out += ',';
            out += String(acc.ay, digitsACCprecision);
            out += ',';
            out += String(acc.az, digitsACCprecision);
            out += '\n';
        }

        if (ROL)
        {
            out += "ROL ";
            out += String(roll, digitsROLprecision);
            out += '\n';
        }

        if (PIT)
        {
            out += "PIT ";
            out += String(pitch, digitsPITprecision);
            out += '\n';
        }

        if (SRO)
        {
            out += "SRO ";
            out += String(rollCommand, digitsSROprecision);
            out += '\n';
        }

        if (SPI)
        {
            out += "SPI ";
            out += String(pitchCommand, digitsSPIprecision);
            out += '\n';
        }

        if (THR)
        {
            out += "THR ";
            out += String(throttleCommand, digitsTHRprecision);
            out += '\n';
        }

        writeChunked(out);
    }
};
