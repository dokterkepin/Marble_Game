### **Project Overview**



This project controls a 'robot-leg'-build out 4 servos using computer vision and manual keyboard input.



Automatic mode: Detects and tracks a pingpong-ball using OpenCV and adjusts servo positions via PID control.



Manual mode: Allows the user to control servo angles directly through keyboard input.



All communication with the Raspberry Pi Pico happens via a serial interface.



### **File Descriptions**



#### main.py

Launches both the automatic detection and manual control threads.

Starts two threads:

* detection.run\_detection\_loop() – runs automatic object detection.
* manual.run\_manual\_loop() – listens for manual keyboard inputs.







#### detection.py

Uses OpenCV to capture video and detect orange or red objects.

Applies PID controllers to calculate servo angles based on object position.

Sends angle commands to the microcontroller via serial port.



#### manual.py

Uses pynput to detect keyboard input.

Allows manual adjustment of servo angles:

* Arrow keys → Servo 1 \& Servo 2
* W/S → Servo 3
* A/D → Servo 4

Sends updated angles to the servo controller through serial communication.



#### servo.py

Runs on the Raspberry Pi Pico with MicroPython.

Reads serial input and moves four servos based on received commands.



### Requirements

#### PC Side

Install the following dependencies:

* opencv-python
* numpy
* pyserial
* pynput



Connect camera and Raspberry Pi pico to your PC and adjust the ports in the source code of the 'detection.py' file.



#### Raspberry Pi pico Side

Connect all four servos to the powersupply and to the GPOs of the Rasperry Pi pico. Adjust the ports in the code of the 'servo.py' file recording to which ports you connected the servos on the pico. Make sure the pico and the powersupply share the same GND.



### How to Build and Run

#### 1\. Run Servo Controller

Upload and run 'servo.py' on the Rasperberry Pi pico.



#### 2\. Run the PC Controller

Run 'main.py' on your PC. This will start the detection and the Manual Control thread at the same time. When only one Control mode is needed either run 'detection.py' or 'Manual.py'.

Note: Since our algorithm relies on a color detection you might have to adjust the HSV Color thresholds in 'detection.py' to match the lighting and the color of your maze/Pingpong ball.  It is crutial that the detection works accuratly in order to Control the robot leg. For development we used a red maze and an orange pingpong-ball.

