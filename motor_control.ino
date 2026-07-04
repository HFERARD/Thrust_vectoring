#pragma once
#include "sensor.h"
#include "controller.h"
// #include "com_ble.h"
#include "com.h"
#include "servo.h"
#include "complementary_filter.h"
#include "radio_control.h"
#include <tuple>

GyroscopeAccelerometer imu;
RadioController radioController;

ServoControllerPID servoRoll(4, 15, -15);  // Servo on pin 4; PID output is a +/-15 correction added to the 90° neutral
ServoControllerPID servoPitch(5, 15, -15); // Servo on pin 5; PID output is a +/-15 correction added to the 90° neutral
OpenBrushlessController brushless(9);      // Brushless on pin 9; PID output is a 0-100% throttle percentage

float rollCommand = 0, pitchCommand = 0, throttleCommand = 0; // Default command values for roll and pitch

SerialHandler SerialHandler(imu, servoRoll, servoPitch, brushless);
espp::ComplementaryFilter preFilter;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT); // Set the built-in LED pin as an output

  digitalWrite(LED_BUILTIN, HIGH); // setup is in progress

  Serial.begin(115200);

  imu.begin();

  preFilter.zeroFromAccel(imu.accRestX, imu.accRestY, imu.accRestZ);

  radioController.begin();

  brushless.begin();
  servoRoll.begin();
  servoPitch.begin();

  digitalWrite(LED_BUILTIN, LOW); // Turn off the built-in LED to indicate setup is complete
}

float roll = 0, pitch = 0;

void loop()
{
  // MAIN LOOP ----- EVERYTHING HAPPENS HERE -----

  imu.update();

  preFilter.update(imu.acceleration, imu.rotationVelocity);
  roll = preFilter.get_roll();
  pitch = preFilter.get_pitch();
  roll = 0;
  pitch = 0;

  std::tie(rollCommand, pitchCommand, throttleCommand) = radioController.getCommand();

  SerialHandler.updateReader();

  // Control servos based on command values and sensed outputs
  // DEBUG IN PROGRESS

  servoRoll.control(rollCommand + 90, 90);
  servoPitch.control(pitchCommand + 90, 90);
  Serial.println("Roll Command: " + String(rollCommand) + ", Pitch Command: " + String(pitchCommand) + ", Throttle Command: " + String(throttleCommand));

  // servoRoll.servo.control(rollCommand + 90);
  // servoRoll.servo.control(pitchCommand + 90);

  brushless.control(throttleCommand);

  SerialHandler.sendData(roll, pitch, rollCommand, pitchCommand, imu.acceleration, imu.rotationVelocity, throttleCommand, servoRoll.pid.control_signal, servoPitch.pid.control_signal);
}
