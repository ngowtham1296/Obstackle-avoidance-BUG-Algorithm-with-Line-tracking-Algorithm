﻿# **PROJECT TITLE: MOBILE ROBOT MOTION CONTROL**This project mainly focuses on the bug algorithm of a Mobile robot (Alpha Bot). In this attempt we used a OPENCV library to make the robot to clear the maze. To make this happen the robot is equipped with Raspberry pi microcontroller, Ultraviolet, IR sensors and camera sensors. The Raspberry Pi is connected with the Linux system via Ethernet. We can activate the code remotely from our desktop using Raspberry pi Linux terminal.**Getting Started:**1. **Prerequisites:** - ALPHA BOT- MOBILE ROBOT - Raspberry pi programming - Feedback Control system - OPEN CV library2. **Installation:** - Assemble the Mobile bot as per the instructions given in      **Link:** [https://www.waveshare.com/w/upload/c/c7/AlphaBot-User-Manual.pdf](https://www.waveshare.com/w/upload/c/c7/AlphaBot-User-Manual.pdf) - Connect the raspberry pi board to the PC to activate the remote Linux desktop      **Link:** [https://maker.pro/raspberry-pi/tutorial/how-to-connect-a-raspberry-pi-to-a-laptop-display](https://maker.pro/raspberry-pi/tutorial/how-to-connect-a-raspberry-pi-to-a-laptop-display) - Install Open CV2      In the linux terminal            1. conda create -n opencv python=3.8            2. conda activate opencv            3. pip install opencv -python   - To verify the installation:                             1. python               2. \_\_version\_\_ - Install yaml     In the linux terminal            1. pip install pyyaml3. **Run:** - Make a circuit with a black tape of thickness not less than 1 inch for the robot to travel in the form of a maze. - After the setup of the mobile robot and the circuit, run the main\_code.py in the terminal window of Linux remote desktop. - After clicking the run option, make the robots IR sensor to face the black tape for at least 5 seconds to calibrate the sensors. - After this place the robot on the track of black tap. Then the robot uses the camera sensor to sense the ARUCO marker and retrieves the distance co-ordinates from the marker. After this the robot tries to move towards the location by clearing the maze using BUG 1 and BUG 2 algorithm.