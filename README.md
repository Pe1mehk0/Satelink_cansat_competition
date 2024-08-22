## Hi everyone,

This is the code for the Satelink team's project in the CanSat competition 2023/2024.

## 🛰️ What is CanSat:

CanSat is an international challenge organized by the European Space Agency in which young people construct simulators of space probes and conduct scientific research using them.

## 📡 Our project:

Primary mission:

CanSat (satelite) will monitor its altitude while falling using a BMP 280 temperature and pressure sensor; connected to the Adafruit Feather M0, the sensor will activate and using the pressure – temperature correlation formula.
At 1Hz, it will transmit the readings once per second through a 433Mhz LoRa RFM96 radio module with an omnidirectional antenna. The data will then be graphed in a real time and saved on a computer.

Secondary mission:

Our CanSat establishes a two-way communication link with the ground station, using Lora RFM96 modules on both ends. It receives information from the ground station when not transmitting primary mission data, saving it on a micro-SD card. 
Commands are then relayed to a rover equipped with a directional receiver antenna, allowing task execution. 

## 📊 Code:

[CanSat code](Final_code_can.ino) - This file contains the code for the CanSat, enabling it to transmit, receive, and store data.

[Ground Station code](Final_code_gs.ino) - This code is designed for a Feather M0 with LoRa RFM96, which is stationed on the ground. 
It facilitates the reception, transmission, and storage of data.

[Interface code](main.py) - This code provides a user interface that displays real-time data graphs, including 
altitude, pressure, temperature, RSSI, and speed. It also allows users to input commands, which are then transferred 
to the Feather M0 for further transmission.

[Rover code](Final_code_rover.ino) - This code is for the rover, enabling it to receive and execute commands, as well as gather data from sensors and save them on SD card. 


## 🔮 Links:
Website - [Satelink](https://satelinkcansat.pl)
