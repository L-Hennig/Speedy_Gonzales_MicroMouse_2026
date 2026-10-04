Speedy Gonzales

This repository contains the code, hardware files, and documentation for the Micromouse robot developed for the 2026 Semester 2 UQMARS Micromouse Competition.

The repository was actively used during development and competition for testing, iteration, and file management. This README provides an overview of the robot design, hardware modifications, and software architecture.
Previous code used still exist. Micromouse_C_code3 was the one used for the competition.

Below is an image of our Micromouse:

<img width="576" height="768" alt="image" src="https://github.com/user-attachments/assets/cb62d371-f0d1-4a5b-9522-f4b3fb0a978b" />

Our design is heavily based on the UQMARS starter Micromouse platform:

🔗 https://github.com/uqmars/starter-micromouse

The original repository contains:

PCB design files
Chassis CAD files
Starter parts list
Modifications We Made

We modified the following subsystems:

Wall detection sensors
Maze solving algorithm

Wall Detection Sensors:
We replaced the IR sensors with VL53L4CD Time of Flight (TOF) Sensors from Core Electronics

🔗 https://core-electronics.com.au/catalogsearch/result/?q=POLOLU-3692

Three sensors were used (front, left, right).

To mount them, we designed and 3D-printed custom holders:
INSTERT STL LINK

To wire up the new sensors we needed an extra breadboard because we did not have enough time to solder the SDA and SCL wires together.


Cost Breakdown:
<img width="1394" height="542" alt="image" src="https://github.com/user-attachments/assets/f50595ae-67aa-4261-8840-a9dc3092762c" />

PCB is same as in Micromouse Starter Kit


