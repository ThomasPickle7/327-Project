# 327-Project
## Overview
This is a repository for documenting an embedded systems by Thomas Pickell and Noah Villa Our project aims to detect when a person has fallen and sned a message via Bluetooth to a mobile device. The device is intended for applications in elder care, similar to products like LifeAlert, and aims to provide an automatic resonse to emergency situations.

## USE
Arduino: to run our Arduino implementation, simply downlaod the code and upload it to any distrubition of an Arduino Uno R3, setting up your arduino board according to the following pinout:

Code Composer Studio: to run our CCS implementation, simply downlaod the file main.c and upload it to a MSP430FR2433 board, setting up your board according to the following pinout:

## DEMONSTRATION:
A Live demonstration of our project can be found [here](https://drive.google.com/file/d/1XZIgEVxlYv1pwuqYM_hLLPm6eGMVjQot/view).

# Components
## I2C
I2C is a serial communications protocol that allows for transmission and reception of data over a clock and data line. DataFrames are sent in 8-bit chunks, along with an address and some logistical information.
### MPU6050
The MPU6050 is a 6-axis gyroscopic sensor and accelerometer, which we use to measure the positional data for our project. To do this, we send messages that include the MPU's address, as well as the register we wish to read. To configure the gyro, we do a similar process, but include a bitmask with which to write to the specified address.

## Bluetooth

# Process
We began by dividing the work into 2 main focuses: configuring the Bluetooth module and establishing an I2C connection. This process was done solely in Code Composer Studio, with all code written in C. During this process, we encountered a series of issues and resolved to implementing the poject in Arduino first, then porting it to CCS when we had a functional prototype.

