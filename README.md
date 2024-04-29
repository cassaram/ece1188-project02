# ECE 1188 Project 2 - Wall-Following Race

This repository contains a full Code Composer Studio project which implements robot control via Bluetooth, MQTT logging, wall-following with the TI-RSLK distance sensor and bump sensors. Information on using the project is included below

## Bluetooth Commands

All commands are sent as UTF-8 / ASCII strings.  
`g` - Starts the robot.  
`s` - Stops the robot.  

## Quick-Start Guide

1. Import this full repository as a Code Composer Studio project
2. Ensure that the `..\inc` folder is placed in the same directory as this project's root folder. The `..\inc` folder can be found with the example projects from Texas Instruments. This is typically at the `C:\ti\tirslk_max_1_00_02\inc` directory.
3. Update the SSID and password in `main.c` to reflect a valid SSID
4. Compile and build the project in Code Composer Studio
5. Flash the project onto the robot.
6. Start the robot
7. Wait for the robot to connect to Wi-Fi
8. Connect to the bluetooth module. Send the command `g` (ASCII) to start the program.
9. Send the command `s` (ASCII) over bluetooth to stop the robot at any time.