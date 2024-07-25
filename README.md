# ESD_Robot

A PyBullet-based simulator using custom hardware to scan/probe objects.

See docs/ for useful information regarding using the machine / using the software.

# Introduction
This project aimed to create surface charge mappings of dielectric objects charged through static charge. The robotic manipulator was a 6 (+1) DoF robot with a probe capable of measuring surface potential. This program combined the free axes' ability to create a movement plan to scan any dielectric object within the bounds (~8-inch cube). The device was armed with a laser displacement sensor, with a combination of linear rails, that could scan any object into the simulation. The entire robotic device was placed within a humidity and temperature controlled environment.

This program implements an algorithm to take any arbitrary point cloud and generate a 'probe plan.' To maximize user input, the user could define charging, discharging, scanning, and sleeping periods.

![image](https://github.com/user-attachments/assets/dd573ee2-cd06-40e2-b363-203feb743d68)
![image](https://github.com/user-attachments/assets/e69b782c-f98b-469b-85f5-61b22e8b5aca)


# Scan Flow
As this software is an 'all-in-one' scanning tool, each object must be processed. A specialized wizard was created to guide the user through importing any object (.stl/.obj, primitive, or scan) and generate a point cloud & probe plan. 


## Generation of Point Clouds
![image](https://github.com/user-attachments/assets/443631c4-1219-428b-aec1-9dd4a604620f)

## Probe Flow Creation Example
https://github.com/user-attachments/assets/f0255179-d4d9-4ff3-8f8c-cd1e98a9dee1

## User Interface
![image](https://github.com/user-attachments/assets/2e8c000e-3886-43ff-85ac-bd551fc9ae02)

# Results + Publication
The results of this project were published for the 2023 IEEE Pulsed Power Conference (PPC): https://ieeexplore.ieee.org/document/10310963

## Inside ESD Machine
![image](https://github.com/user-attachments/assets/f9a44c13-bdb1-4080-a861-eaa6693a85e7)

## ESD Machine Hardware
![image](https://github.com/user-attachments/assets/05c77b68-6846-4647-bb2b-565446099a43)
