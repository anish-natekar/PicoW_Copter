#include<Wire.h>

#define I2C_CLK_FREQ 400000 // 400kHz
const u_int8_t IMUAddress = 0x68;  // Address for MPU6050 IMU sensor
// IMU offset 
int16_t gyroXoffset = 0;  
int16_t gyroYoffset = 0;  
int16_t gyroZoffset = 0;  
int16_t accXoffset = 0;   
int16_t accYoffset = 0;   
int16_t accZoffset = 0;   
// MPU6050 IMU 
int16_t accX, accY, accZ; // accelerometer
int16_t tempRaw;  
int16_t gyroX, gyroY, gyroZ; // gyroscope
float temp; // temperature
int prev;   // keeps track of time before reading IMU data

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // set the built in LED pin as Output
  Serial.begin(115200);
  Wire.setClock(I2C_CLK_FREQ);  // setting I2C communication frequency to 400kHz
  Wire.begin(); // starting I2C communication over SDA0 and SCL0 pins
  
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(IMUAddress);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x08); // +- 1000 degrees/s
  Wire.endTransmission();

  Wire.beginTransmission(IMUAddress);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x10); // +- 16g
  Wire.endTransmission();

  Wire.beginTransmission(IMUAddress);
  Wire.write(0x1A); // CONFIG
  Wire.write(0x03);
  Wire.endTransmission();

  // IMU offset calculation
  int offcnt;
  long gx=0, gy=0, gz=0, ax=0, ay=0, az=0;  // variables to store sum of 1000 readings
  for(offcnt=0; offcnt<=1000; offcnt++) {
    // Reading IMU data 1000 times to calculate offset values of IMU
    Wire.beginTransmission(IMUAddress);
    Wire.write(0x3B); // GyroXhigh byte
    Wire.endTransmission();
    Wire.requestFrom(IMUAddress, 14); // request 14 bytes of data from IMU
    while(Wire.available() < 14); // If we have received 14 bytes exit out of loop
    // read IMU data values
    accX = Wire.read()<<8|Wire.read();
    accY = Wire.read()<<8|Wire.read();
    accZ = Wire.read()<<8|Wire.read();
    tempRaw = Wire.read()<<8|Wire.read();
    gyroX = Wire.read()<<8|Wire.read();
    gyroY = Wire.read()<<8|Wire.read();
    gyroZ = Wire.read()<<8|Wire.read();   
    // Sum the values read from IMU
    gx += gyroX;
    gy += gyroY;
    gz += gyroZ;
    ax += accX;
    ay += accY;
    az += accZ;
    delay(3); // simulating delay for rest of the quadcopter processes
    if(offcnt%40 == 0)    
      digitalWrite(LED_BUILTIN, HIGH);  // LED blinks to indicate offset calculation is going on
    else
      digitalWrite(LED_BUILTIN, LOW);
  }  
  // get the average of 1000 readings
  gyroXoffset = (int16_t)(gx/1000);
  gyroYoffset = (int16_t)(gy/1000);
  gyroZoffset = (int16_t)(gz/1000);
  accXoffset = (int16_t)(ax/1000);
  accYoffset = (int16_t)(ay/1000);
  accZoffset = (int16_t)(az/1000);
}

void loop() {
  prev = micros();  // record time when we started reading IMU data
  // read IMU values
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(IMUAddress, 14);

  while(Wire.available() < 14);
   
  accX = Wire.read()<<8|Wire.read();
  accY = Wire.read()<<8|Wire.read();
  accZ = Wire.read()<<8|Wire.read();
  tempRaw = Wire.read()<<8|Wire.read();
  gyroX = Wire.read()<<8|Wire.read();
  gyroY = Wire.read()<<8|Wire.read();
  gyroZ = Wire.read()<<8|Wire.read();   
  // subtract offset from readings 
  gyroX -= gyroXoffset;
  gyroY -= gyroYoffset;
  gyroZ -= gyroZoffset;
  accX -= accXoffset;
  accY -= accYoffset;
  accZ -= accZoffset;
  temp = (float)tempRaw;
  // print data
  //Serial.printf("AccX = %d, AccY = %d, AccZ = %d, Temp = ", accX, accY, accZ);
  //Serial.print(temp);  
  //Serial.printf(", GyroX = %d, GyroY = %d, GyroZ = %d\n", gyroX, gyroY, gyroZ);
  Serial.printf("Time = %d\n", micros() - prev);
  delay(250);
}
