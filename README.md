# morgan-firmware
## MORGAN (MOdel Rocket GuidANce and Control) Firmware for rocket flight

#### A Firmware for the MORGAN rocket flight computer.

![Morgan Board](img/Morgan_Board.jpg ':size=300')

It was built with the following requirements in mind:

* Fast MCU, plenty of GPIO, Wifi & BLE, relatively low power (we chose the ESP-32 Microcontroller)
* Modular code design
* Attitude stabilisation (Pitch/Yaw) using either canards fins and/or Thrust Vectoring Control (TVC) and soon, Reaction Wheel (for roll control).
* Up to 4 servos connections independently controlled using PID loops.
* Altitude / Apogee detection using BMP-280 and MPU-6050 for Gyro/Axcel.
* 4 pyros channels (using mosfets) for deployment of recovery systems and stage separation.
* Must be able to log as much information from the flight passible.
* On board 64Mb Flash memory + optional SD Card for data logging.
* Battery voltage monitoring (regulated from 7~12v down to 5v and 3.3v).
* GPS capability (not used yet)
* ~~~Bluetooth bi-directional communication for parameters configuration and pre-flight systems tests.~~~
* WIFI bi-directional communication for parameters configuration and pre-flight systems tests.
* And the all important flashing RGB Led and Piezzo Buzzer :)


## ~~~Bluetooth~~~ WebSocket WIFI WEB App
A Web ~~~Bluetooth~~~ app is used to configure the MORGAN flight computer. Its project page is https://github.com/intensite/morgan-app

The Bluetooth portion has been deprecated for now had it had stability problems that was crashing the flight computer.  The code was commented out for future rework if necessary.


## Note:

The project was moved/renamed from https://github.com/intensite/rocketcontrol

The main reason was a change of developpment environment. The project is now developped using platformIO wich greatly speedup compile times and library management.
The old code will stay here to preserve the commit history.