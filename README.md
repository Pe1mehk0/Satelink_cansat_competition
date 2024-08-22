## Hi everyone,

This is the code for the Satelink team's project in the CanSat competition.

## 🛰️ What is CanSat:

CanSat is an international challenge organized by the European Space Agency in which young people construct simulators of space probes and conduct scientific research using them.

## 📡 Our project:

Primary mission:

Our CanSat (satelite) will monitor its altitude while falling using a BMP 280 temperature and pressure sensor; connected to the Adafruit Feather M0, the sensor will activate and using the pressure – temperature correlation formula.
At 1Hz, will transmit the readings once per second through a 433Mhz LoRa RFM96 radio module with an omnidirectional antenna. The data will then be graphed out to provide us with the descent rate. 
Knowing the crosswind value and the rate of change of altitude, we will be able to better estimate the position of the can as it falls on the ground.

Secondary mission:

Our CanSat establishes a two-way communication link with the ground station, using Lora RFM96 modules on both ends. It receives information from the ground station when not transmitting primary mission data, saving it on a micro-SD card. 
Commands are then relayed to a rover equipped with a directional receiver antenna, allowing task execution. 

## 📊 Code:

CanSat:
[CanSat code](Final_code_can.ino) - This file stands for a CanSat's code which allows to send, receive and save data. 

Ground Station:
[Ground Station code](Final_code_gs.ino) - This code stands for a Feather M0 with LoRa RFM96 which is placed on the ground station to receive, send and save data.

[Interface code](main.py) - This code stands for the users interface which allows to display life-time data graphs with Altitude, Pressure, Tempreture, RSSI and Speed. Also it allows to type_in commands and transfer them to Feather M0 for further sending.

Rover code:
[Rover code](Final_code_rover.ino) - This code stands for a rover which receives and executes commands, and gather information from sensors. 

---
## 🔮 Links:
Website - [![](https://satelinkcansat.pl)
