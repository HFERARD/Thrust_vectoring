#pragma once
#include "sensor.h"
#include "controller.h"
#include "serial_com.h"
#include "servo.h"
#include "complementary_filter.h"

GyroscopeAccelerometer imu;
ServoControllerPID servoRoll(2, 90, -90);  // Servo on pin 2; PID output is a +/-90 correction added to the 90deg neutral
ServoControllerPID servoPitch(3, 90, -90); // Servo on pin 3; PID output is a +/-90 correction added to the 90deg neutral

float rollCommand = 0, pitchCommand = 0; // Default command values for roll and pitch

SerialHandler serialHandler(imu, servoRoll, servoPitch);
espp::ComplementaryFilter preFilter;

void setup()
{
  sensor::setup();
}

float roll = 0, pitch = 0; // Initialize roll and pitch angles

void loop()
{
  // MAIN LOOP ----- EVERYTHING HAPPENS HERE -----
  roll = preFilter.get_roll();
  pitch = preFilter.get_pitch();

  preFilter.update(imu.acceleration, imu.rotationVelocity);

  // Update sensor readings
  imu.update();
  serialHandler.updateReader();
  serialHandler.sendData(roll, pitch, rollCommand, pitchCommand, imu.acceleration, imu.rotationVelocity);

  // Control servos based on command values and sensed outputs

  servoRoll.control(rollCommand + 90, roll + 90);
  servoPitch.control(pitchCommand + 90, pitch + 90);
}
