# Project Ideas

## Fall Sensor:
- **Description**: A device that can detect when a person has fallen and alert someone.

### Process of working:
- Tracks the person's movement using a gyroscope and accelerometer.
- Checks changes in velocity and acceleration.
- If a fall is detected, sends a message to a phone
- Goes to Low-power mode when the velocity isn't changing, have an LED to show

### Hardware Reqs:
- Gyro/Accelerometer: MPU6050 (I2C)
- Plastic Casing/ Protection (3D Printed)
- Bluetooth Communication: HC-05 (UART)
- Microcontroller: MSP-EXP430FR2433 https://www.mouser.com/ProductDetail/Texas-Instruments/MSP-EXP430FR2433?qs=j%252B1pi9TdxUbUYSWQHmscvw%3D%3D

# Things to look into
- Intefacing with mpu6050 in C (I2C comms, check the datasheet)
- Interfacing with HC-05 in C (UART, also check the datasheet)
- Interfacing with bluetooth (IE how do we make a notification appear?)
- see if theres anything about the new MSP baord thats useful
- COnsider algorithm for determining 'non-regular velocities'
- generally figure out exactly how our project will work.


# Things to do:
- Fill out parts list
- Verify that we dont need more than whats on the parts list
- look into the stuff above

# stuff we want the code to do:
- track velocity and acceleration
- if the change in velocity is severe, throw some flag
- if the flag is thrown, send a message via bluetooth
- if the flag isn't thrown, go into a low power mode (not sure which yet)


# Resources

Project Proposal: Fall Sensor

Introduction: Our project aims to develop a fall sensor device that can detect when a person has fallen and alert someone. The device will utilize a gyroscope and accelerometer to track the person's movement and analyze changes in velocity and acceleration. In case of a fall detection, the device will send a message to a connected phone via Bluetooth communication. Additionally, the device will have a low-power mode when the velocity isn't changing, indicated by an LED.

Splitting Tasks: To efficiently manage the project, we propose dividing the tasks between two potential partners, Thomas and Partner2.

Thomas Responsibilities: Thomas will focus on the hardware aspects of the project. This includes researching and interfacing with the MPU6050 gyro/accelerometer using I2C communication. They will also be responsible for designing and 3D printing a plastic casing for the device, ensuring its protection. Furthermore, Thomas will explore the functionalities of the MSP-EXP430FR2433 microcontroller and its compatibility with the project requirements.

Noah Responsibilities: Noah will primarily handle the software aspects of the project. They will research and implement the communication with the HC-05 Bluetooth module using UART. Noah will also investigate how to make a notification appear on the connected phone when a fall is detected. Additionally, they will develop an algorithm to determine non-regular velocities and implement the tracking of velocity and acceleration. Finally, Noah will work on the low-power mode functionality of the device.

By dividing the tasks in this manner, Thomas and Noah can work concurrently and efficiently towards the successful completion of the fall sensor project.