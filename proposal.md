# Project Proposal: Fall Sensor

Our project aims to develop a fall sensor device that can detect when a person has fallen and alert someone. The device will utilize a gyroscope and accelerometer to track the person's movement and analyze changes in velocity and acceleration. In case of a fall detection, the device will send a message to a connected phone via Bluetooth communication. Additionally, the device will have a low-power mode when the velocity isn't changing, indicated by an LED.

To efficiently manage the project, we propose dividing the tasks between the members of our group, Thomas Pickell and Noah Villa.

Thomas will focus on the hardware aspects of the project. This includes researching and interfacing with the MPU6050 gyro/accelerometer using I2C communication. They will also be responsible for designing and 3D printing a plastic casing for the device, ensuring its protection. Furthermore, Thomas will explore the functionalities of the MSP-EXP430FR2433 microcontroller and its compatibility with the project requirements.

Noah will primarily handle the software aspects of the project. They will research and implement the communication with the HC-05 Bluetooth module using UART. Noah will also investigate how to make a notification appear on the connected phone when a fall is detected. Additionally, they will develop an algorithm to determine non-regular velocities and implement the tracking of velocity and acceleration. Finally, Noah will work on the low-power mode functionality of the device.

By dividing the tasks in this manner, Thomas and Noah can work concurrently and efficiently towards the successful completion of the fall sensor project.