# Thrust vectoring project 

_Hackathon Mines Paris PSL 1A_<br>
**Date** : 29/06/2026 - 3/07/2026 <br>
**Teacher** : Théo Akbas <br>
**Group** : Arthur AULLIANS, Yannis AULLEN-CHOUBRAC, Titouan DROUYNOT, Gaspard CHUPIN, Hector FERARD <br>

## Introduction

The objective of this project is to design and build a thrust vectoring drone : a system able to fly and self-balance using a single propeller. To do so, we had to conceive both the hardware and the software 

### Setup 

To run this code, you need our Arduino Nano BLE 33 board with its hardware. Libraries used in this code (in test files and to run the ```.ino```file) are (to be installed through the Arduino IDE 2.3.10) : 
- ```ArduinoBLE``` : enables contact with the Nano BLE 33 circuit
- ```Arduino_LSM9DS1``` : enables access to the IMU readings (IMU model LSM9DS1 on the BLE33, for the BLE 33 Sense Rev2 install ```Arduino_BMI270_BMM150```)
- ```IBusBM```: IBUS protocol for receiving telemetry from RC controller. 



### Design choices 

We chose to use an Arduino Nano BLE 33 (Sense Rev2/ simple as we had two circuits) as the backbone of the electronics. All the on-board software is coded in C++, and the computer-based user interface which collects data is vibe-coded in Python. 

### Results and opportunities

Although we didn't manage to make the drone fly safely and in a stable manner, we managed to combine software and hardware and command the 2-axis thrust vectoring system. However, the controllers have not been tuned and this represents the next step of the project. 

## Hardware 

The hardware consists of a 2 axis servo-controlled motor handler, on top of which we fixed a brushless motor to power the propeller. A RC-controller is connected through UART (with a PWN signal) to the Serial1 channel, enabling the board to get information from the user. The board then sends data through the microUSB port on the Serial channel. 

We commanded the brushless motor with a an ESC PCB, which enabled the command to be the same a sthe one for servos : a PWN signal with duty cycle defining rotor speed. 


## Software 

The on-board is completely implemented in C++ with OOP structure. Although this choice could be discussed, it enabled clear defintion in ```setup``` and use cases in the main ```loop```. The idea is that, thanks to the ESC card, all motors are controlled thanks to a PWN (Pulse-Width Modulation between 1000 and 2000 microseconds) so it can be interesting to implement similar code for all motors. 

When adding functionaliies like the RC command or the complementary filter, used to convert IMU acceleration and gyroscopic data to an estimate of drone pitch and roll, we only needed to create a new class and add a new line to the main loop. 

The control system is based on a closed pitch and roll-controlled loop. The command in pitch and roll is compared to sensed complementary-filter-outputted real pitch and roll, and the error are sent into a PID corrector (which are both yet to be tuned) to then feed the axis-controlling servo motors. The throttle output is controlled open-loop but one of the next steps of the project is to implement altitude-controlled closed-loop throttle command (with another PID, estimates of the altitude being sent by a laser distance sensor).

The computer-side UI was coded completely in Python with AI assistance (Claude Sonnet + Opus) , which analysed our ```com.h``` to create interfaces between user decision and transmission through Serial protocol to the board. The functional file is the ```ui_withdebug_serial.py```file which contains code with debug logs and using Serial for communication. 
