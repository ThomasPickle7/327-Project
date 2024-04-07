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
- HC-05 interfacing with MSP430:
https://circuitdigest.com/microcontroller-projects/interfacing-hc-05-bluetooth-module-with-msp430-launchpad-to-control-an-led
