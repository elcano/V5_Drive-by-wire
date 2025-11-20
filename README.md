# V5_Drive-by-wire
Updated code and PCB for light vehicle drive by wire.

This repository is for use with the **Drive by Wire** board created to be used for the ELCANO self-driving tricycle. It controls throttle (Ebike controller), brakes (solenoid) and steering (either linear servo or motor controller board with a linear actuator). All vehicle actions are logged to an SD card, which is the main data for debugging. Vehicle can be controlled by

1) 6-channel Radio Control (RC) unit

2) Operator controls (OP)

3) Vehicle automation computer via CAN. We have demonstrated a Jetson Nano using a Pixhawk to find GNSS points and move from waypoint to waypoint.

In order for this module to run external libraries are required.

Libraries needed are listed below:

Version 4 and 5 (Arduino Due)  _______________________________________________
* Due CAN and CAN Common by Collin Kidder
    https://github.com/collin80/due_can
    https://github.com/collin80/can_common


## To Install

* Download the **Drive_By_Wire** module into the project parent directory
```
git clone https://github.com/elcano/Drive-by-wire.git
```
* Move into the **Drive_By_Wire** folder
```
cd Drive_By_Wire
```
* Download the required libraries
```
git clone https://github.com/SweBarre/MCP48x2.git
git clone https://github.com/ivanseidel/DueTimer.git
git clone https://github.com/collin80/can_common.git
git clone https://github.com/collin80/due_can.git
```
* Add the folders above to the IDE directory. In Windows, add the required libraries to: C:\Users\<username>\Documents\Arduino\libraries
* Arduino Due requires additional installation. On the IDE, go to Tools -> Boards -> Boards Manager -> Arduino SAM Boards (32-bits ARM Cortex-M3) 

## Navigating Arduino IDE: 
* **Selecting Board:** Tools -> Board -> Arduino SAM Boards -> Arduino Due
* **Selecting Serial Port:** Tools -> Port -> COM (Arduino Due)
* **Accessing Serial Monitor:** Right-hand Corner -> Select Baud Rate (115200)

## Drive-by-Wire: 
* **Low-level board code with RC control support with existing code for brakes, steering, and throttle.** 
