#pragma once
#include <Servo.h>

// A FAIRE MARDI : CLASSE DE COMMANDE BRUSHLESS AVEC PID EN ALTITUDE

const int ESC_PIN = 9; // ESC signal pin

Servo ESC;

void setup()
{
    ESC.attach(ESC_PIN, 1000, 2000);
}

int Speed;

void loop()
{
    Speed = analogRead(A0);
    Speed = map(Speed, 0, 1023, 1000, 2000);
    ESC.writeMicroseconds(Speed);
}