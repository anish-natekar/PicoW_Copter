#include <Wire.h>
#define BMPAddress 0x76

// Barometer calibration values
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
int16_t dig_P6, dig_P7, dig_P8, dig_P9;

// Altitude variables
float AltitudeBarometer, AltitudeBarometerStartUp;
int RateCalibrationNumber;

// read barometer signal
void barometer_signal() {
  Wire.beginTransmission(BMPAddress);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(BMPAddress, 6);
  uint32_t press_msb = Wire.read(); // 0xF7 
  uint32_t press_lsb = Wire.read(); // 0xF8
  uint32_t press_xlsb = Wire.read();
  uint32_t temp_msb = Wire.read();
  uint32_t temp_lsb = Wire.read();  
  uint32_t temp_xlsb = Wire.read(); // 0xFC

  // Construct raw temperature and pressure measurements
  // msb contans bits [19-12], lsb contains bits [11-4], and xlsb contains bits [3-0]
  unsigned long int adc_P = (press_msb<<12)|(press_lsb<<4)|(press_xlsb>>4);
  unsigned long int adc_T = (temp_msb<<12)|(temp_lsb<<4)|(temp_xlsb>>4);

  // Construct fine resolution temperature value
  signed long var1, var2;
  var1 = ((((adc_T>>3)-((signed long int)dig_T1<<1)))*((signed long int)dig_T2))>>11;
  var2 = (((((adc_T>>4) - ((signed long int)dig_T1)) * ((adc_T>>4)-((signed long int)dig_T1)))>>12)*((signed long int)dig_T3))>>14;
  signed long int t_fine = var1 + var2;

  // Construct the compensated and calibrated pressure p according to manufacturer
  unsigned long int p;
  var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
  var2 = (((var1>>2)*(var1>>2))>>11)*((signed long int)dig_P6);
  var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
  var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
  var1 = (((dig_P3*(((var1>>2)*(var1>>2))>>13))>>3)+((((signed long int)dig_P2)*var1)>>1))>>18;
  var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);
  if(var1 == 0) { p = 0; }
  p = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
  if(p<0x80000000) { p = (p<<1) / ((unsigned long int)var1); }
  else { p = (p / (unsigned long int)var1) * 2; }
  var1 = (((signed long int)dig_P9)*((signed long int)(((p>>3)*(p>>3))>>13)))>>12;
  var2 = (((signed long int)(p>>2))*((signed long int)dig_P8))>>13;
  p = (unsigned long int)((signed long int)p + ((var1 + var2 + dig_P7)>>4));

  double pressure = (double)p/100;  // pressure in hPa
  AltitudeBarometer = 44330*(1-pow(pressure/1013.25, 1/5.255))*100; // Altitude in cm
}

void setup() {
  // BMP280 setup
  Serial.begin(57600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  // Setup BMP280 barometer optimized for indoor navigation
  Wire.beginTransmission(BMPAddress);
  Wire.write(0xF4); // measurement register setup for indoor mode
  Wire.write(0x57); // normal mode, pressure oversampling x16 temperature oversampling x2
  Wire.endTransmission();   
  // Setup configuration register
  Wire.beginTransmission(BMPAddress);
  Wire.write(0xF5);
  Wire.write(0x14); // IIR filter coefficient 16 rest are 0
  Wire.endTransmission();
  // Importing 12 trimming parameters from sensor
  uint8_t data[24], i = 0;
  // First trimming parameter
  Wire.beginTransmission(BMPAddress);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(BMPAddress, 24); // 24 bytes of data from register 0x88 to 0x9F
  while(Wire.available()) {
    data[i] = Wire.read();
    i++;
  }    
  dig_T1 = (data[1]<<8) | data[0];
  dig_T2 = (data[3]<<8) | data[2];
  dig_T3 = (data[5]<<8) | data[4];
  dig_P1 = (data[7]<<8) | data[6];
  dig_P2 = (data[9]<<8) | data[8];
  dig_P3 = (data[11]<<8) | data[10];
  dig_P4 = (data[13]<<8) | data[12];
  dig_P5 = (data[15]<<8) | data[14];
  dig_P6 = (data[17]<<8) | data[16];
  dig_P7 = (data[19]<<8) | data[18];
  dig_P8 = (data[21]<<8) | data[20];
  dig_P9 = (data[23]<<8) | data[22];
  delay(250);
  // barometer calibration calculating altitude reference level
  for(RateCalibrationNumber = 0; RateCalibrationNumber<2000; RateCalibrationNumber++) {
    barometer_signal();
    AltitudeBarometerStartUp += AltitudeBarometer;
    delay(1);
  }
  AltitudeBarometerStartUp/=2000;
}

void loop() {
  // Read the barometer and print altitudes
  barometer_signal();
  AltitudeBarometer = AltitudeBarometerStartUp;
  Serial.print("Altitude [cm]:");
  Serial.println(AltitudeBarometer);
  delay(50);
}
