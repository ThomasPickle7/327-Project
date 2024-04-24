# 327-Project
This is a repository for documenting an embedded systems by Thomas Pickell and Noah Villa Our project aims to detect when a person has fallen and sned a message via Bluetooth to a mobile device when it does


# Components
## I2C
I2C is a serial communications protocol that allows for transmission and reception of data over a clock and data line. DataFrames are sent in 8-bit chunks, along with an address and some logistical information.
### MPU6050
The MPU6050 is a 6-axis gyroscopic sensor and accelerometer, which we use to measure the positional data for our project. To do this, we send messages that include the MPU's address, as well as the register we wish to read. To configure the gyro, we do a similar process, but include a bitmask with which to write to the specified address.

## Bluetooth
