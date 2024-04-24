#include<Wire.h>;

const int MPU_addr = 0x68;
bool fell = false;
bool print = false;
char Incoming_value = 0;

void setup() {
  Serial.begin(9600);
  Serial.println ("Hit a key to start");     // signal initalization done
  while (Serial.available() == 0);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(50);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);
  Wire.write(0x10);
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void loop() {

  byte XAxis = 0;
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 1, true);
  XAxis = Wire.read();
  byte XAxisL = 0;
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3C);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 1, true);
  XAxisL = Wire.read();
  int16_t XAxisFull = XAxis << 8 | XAxisL;
  float XAxisFinal = (float) XAxisFull / 16384.0;
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3D);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 4, true);
  int16_t YAxisFull =  (Wire.read() << 8 | Wire.read());
  int16_t ZAxisFull =  (Wire.read() << 8 | Wire.read());
  float YAxisFinal = (float)YAxisFull / 16384.0;
  float ZAxisFinal = (float) ZAxisFull / 16384.0;
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x41);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 2, true);
  int16_t tempFull = Wire.read() << 8 | Wire.read();
  float tempFinal = (float) tempFull / 340.0 + 36.53;
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);
  int16_t XGyroFull = Wire.read() << 8 | Wire.read();
  int16_t YGyroFull = Wire.read() << 8 | Wire.read();
  int16_t ZGyroFull = Wire.read() << 8 | Wire.read();
  float XGyroFinal = (float)XGyroFull/32.8;
  float YGyroFinal = (float)YGyroFull/32.8;
  float ZGyroFinal = (float)ZGyroFull/32.8;
  float magnitude = (float) sqrt(sq(ZGyroFinal) + sq(YGyroFinal) + sq(XGyroFinal));
  if(fell == true){
    if(print = false){
    print = true;
    //
    if (Serial.available()  > 0){
      Incoming_value = '100000';
      Serial.println(Incoming_value);
    }
    //
    }
  }else{
    Serial.print("X Axis = ");
    Serial.print(XGyroFinal);
    Serial.println(" deg/s");
    Serial.print("Y Axis = ");
    Serial.print(YGyroFinal);
    Serial.println(" deg/s");
    Serial.print("Z Axis = ");
    Serial.print(ZGyroFinal);
    Serial.println(" deg/s");
    Serial.print("Magnitude: ");
    Serial.println(magnitude);
      //Hell yeah!

      }
    if(magnitude > 200){
      fell = true;
    }
  delay(200);
}