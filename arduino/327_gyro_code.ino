// Integrated code for the MPU6050 and the HC-05 Bluetooth module
// Written by Noah Villa and Thomas Pickell
#include <Wire.h>;
#include <SoftwareSerial.h>

// Define the pins for the Bluetooth module
#define BT_RX 10
#define BT_TX 11

//The default address of the MPU is 0x68
const int MPU_addr = 0x68;
bool fell = false;
bool print = false;

// Create a SoftwareSerial object for the Bluetooth module.
SoftwareSerial BTSerial(BT_RX, BT_TX);

void setup() {
  //Here we will begin serial communications with the gyro and Bluetooth module seperately
  // we give them both the same baud rate of 9600 so we can read messages from both seamlessly
  
  Serial.begin(9600);
  BTSerial.begin(9600);

  // GYRO SETUP
  Wire.begin();
  //Begin transmission with the MPU
  Wire.beginTransmission(MPU_addr);
  // We will write to the PWR_MGMT_1 register (0x6B) and set the value to 0 to wake up the MPU
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(50);
  Wire.beginTransmission(MPU_addr);
  // Here, we write to the GYRO_CONFIG register (0x1B) and set the value to 0x10 to set the full scale range to 500 degrees per second
  Wire.write(0x1B);
  Wire.write(0x10);

  //End and restart transmission
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_addr);
  // We write to the ACCEL_CONFIG register (0x1C) and set the value to 0x00 to set the full scale range to 2g
  Wire.write(0x1C);
  Wire.write(0x00);
  
  Wire.endTransmission(true);
}

void loop() {
  // Here we will read the gyro data from the MPU6050
  Wire.beginTransmission(MPU_addr);
  // Specifies the register we want to read from
  Wire.write(0x43);
  Wire.endTransmission(false);
  // Requests 6 bytes of data from the specified register
  // This includes the higher and lower bits of the X, Y, and Z gyro values
  Wire.requestFrom(MPU_addr, 6, true);
  // From here, we will read the data in and calculate the values for the gyro in degrees per second
  // Here, We shift the first byte 8 bits to the left and then OR it with the second byte to get the full value
  int16_t XGyroFull = Wire.read() << 8 | Wire.read();
  int16_t YGyroFull = Wire.read() << 8 | Wire.read();
  int16_t ZGyroFull = Wire.read() << 8 | Wire.read();
  // We divide the full value by 32.8 to get the gyro value in degrees per second
  float XGyroFinal = (float)XGyroFull / 32.8;
  float YGyroFinal = (float)YGyroFull / 32.8;
  float ZGyroFinal = (float)ZGyroFull / 32.8;
  // Finally, we take the magnitude of the gyro values through the 2-norm
  float magnitude = (float)sqrt(sq(ZGyroFinal) + sq(YGyroFinal) + sq(XGyroFinal));
  // The magnitude is then compared to a threshold value to determine if the user has fallen
  if (magnitude > 300) {
    fell = true;
  }
  // If the user has fallen, we will send a message to the Bluetooth module
  if (fell == true) {
      BTSerial.println("It seems you have fallen! contacting emergency services...");
      // We then set the value of fell to false and wait 2 seconds before checking again
      // This is to reduce the chances of sending multiple messages
      fell = false;
      delay(2000);
  }//The below code is for debugging purposes
  // else {
  //   Serial.print("X Axis = ");
  //   Serial.print(XGyroFinal);
  //   Serial.println(" deg/s");
  //   Serial.print("Y Axis = ");
  //   Serial.print(YGyroFinal);
  //   Serial.println(" deg/s");
  //   Serial.print("Z Axis = ");
  //   Serial.print(ZGyroFinal);
  //   Serial.println(" deg/s");
  //   Serial.print("Magnitude: ");
  //   Serial.println(magnitude);
  // }
  delay(200);
}