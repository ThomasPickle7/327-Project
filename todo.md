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
