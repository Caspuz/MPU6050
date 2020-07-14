#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>
#include "SdFat.h"
#include <SPI.h>

//////////////////////////////////
//      File system object      //
//////////////////////////////////
SdFat sd;
SdFile dataFile;
#define FILE_BASE_NAME "Data"
#define ARRAY_LENGTH 25
#define MEASUREMENT_TIME 20000
#define BUFFER_SIZE 512
char fileName[13] = FILE_BASE_NAME "00.dat";

//////////////////////////////////
//        IMU Definition        //
//////////////////////////////////
MPU6050 mpu;

//////////////////////////////////////////
//   Error messages stored in flash.    //
//////////////////////////////////////////
#define error(msg) sd.errorHalt(F(msg))

//////////////////////////////////////////
//          Other definitions           //
//////////////////////////////////////////
long int flush_interval = 200;
long int prev_flush = 0;
int lines = 0;
long unsigned start_time;


struct dataStore {
  unsigned long timestamp;
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  int16_t gyrX;
  int16_t gyrY;
  int16_t gyrZ;
};


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  Wire.begin();
  Wire.setClock(400000L);
  Serial.begin(250000);
  while (!Serial);                    // wait until finished
  Serial.print(F("Initializing I2C devices..."));
  mpu.initialize();
  delay(500);
  // verify connection
  Serial.print(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
//  if (!mpu.testConnection())
//  {
//    while(1)
//    {
//      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
//      delay(100);
//      if(mpu.testConnection())
//      {
//        break;
//      }
//    }
//  }
  Serial.print("Calibrations starting");
  calibration();
  mpu.setFullScaleAccelRange(3);
  mpu.setFullScaleGyroRange(0);
//  Serial.print("Acceleration range: "); Serial.println(mpu.getFullScaleAccelRange());
  Serial.println("");
  Serial.println("Gyroscope calibration complete");

  //  initialize SD card
  Serial.print(F("Initializing SD card..."));
  int chipSelect = 4;
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    sd.initErrorHalt();
  }
  filecreate();                       //create file and write header
  Serial.println(F("SD Initialized, measurements begin"));
  start_time = millis();
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  struct dataStore dataBuffer;
  int i = 0;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    dataBuffer.timestamp = micros();
    dataBuffer.accX = ax;
    dataBuffer.accY = ay;
    dataBuffer.accZ = az;
    dataBuffer.gyrX = gx;
    dataBuffer.gyrY = gy;
    dataBuffer.gyrZ = gz;
    dataFile.write((const uint8_t *)&dataBuffer, sizeof(dataBuffer));

//    if(millis()<10020 && millis()>10000)
//    {
//      Serial.print("ax: ");Serial.print(ax);Serial.print("\t");Serial.print("ay: ");Serial.print(ay);Serial.print("\t");Serial.print("az: ");Serial.println(az);
//    }
//    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
 //   dataFile.print(millis());dataFile.write(',');dataFile.print(ax);dataFile.write(',');dataFile.print(ay);dataFile.write(',');dataFile.print(az);dataFile.write(',');dataFile.print(gx);dataFile.write(',');dataFile.print(gy);dataFile.write(',');dataFile.println(gz);
 //
    lines++;

  if (millis() - prev_flush >= flush_interval)
  {
    prev_flush = millis();
    dataFile.sync();
    //state = !state;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    //digitalWrite(LED_BUILTIN, (state) ? HIGH : LOW);
    //Serial.println("FLUSHED!");
  }
  if(millis()-start_time >= MEASUREMENT_TIME)
  {
    dataFile.close();
    Serial.print("File finished, ");Serial.print(lines);Serial.print(" number of measurements in ");Serial.print((millis()-start_time)/1000);Serial.println(" seconds.");
    delay(20);
    SPI.end();
    digitalWrite(LED_BUILTIN, HIGH);
    delay(20);
    while(1);
  }
}



//////////////////////////////////////////////
//                  FUNCTIONS               //
//////////////////////////////////////////////
void filecreate() {
  Serial.print("Creating file...");
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  // Find an unused file name.
  if (BASE_NAME_SIZE > 6) {
    error("FILE_BASE_NAME too long");
  }
  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    }
    else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    }
    else {
      error("Can't create file name");
    }
  }
  if (!dataFile.open(fileName, O_CREAT | O_WRITE)) {
    error("file.open");
  }
  //dataFile.println(F("Time,AccX,AccY,AccZ,RotX,RotY,RotZ"));
  //dataFile.sync();
  Serial.print(fileName); Serial.println(" created");
}

void calibration() {
  // Calibrate gyro
  int i = 5, itterations = 100;
  int16_t cal_gx = 0, cal_gy = 0, cal_gz = 0;
  mpu.setXGyroOffset(0);                      //set offsets to 0
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  getMeanRot(&cal_gx, &cal_gy, &cal_gz, itterations);         // Get mean values from gyroscope
  getOffset(&cal_gx, &cal_gy, &cal_gz);

  mpu.setXAccelOffset(-1387);                 // Accel offsets barely change, set vaue
  mpu.setYAccelOffset(-2039);
  mpu.setZAccelOffset(730);
  mpu.setXGyroOffset(cal_gx);                 // Gyro offsets are calibrated each startup above
  mpu.setYGyroOffset(cal_gy);
  mpu.setZGyroOffset(cal_gz);
}

void getMeanRot(int16_t* mean_gx, int16_t* mean_gy, int16_t* mean_gz, int itterations) {
  int i = 0;
  int16_t gx, gy, gz;
  long int buff_gx = 0, buff_gy = 0, buff_gz = 0;

  while (i < (itterations + 101)){
    mpu.getRotation(&gx, &gy, &gz);
    if (i > 100 && i <= (itterations + 100)){
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (itterations + 100)){
      buff_gx = buff_gx / itterations;
      buff_gy = buff_gy / itterations;
      buff_gz = buff_gz / itterations;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
  *mean_gx = (int16_t)buff_gx;
  *mean_gy = (int16_t)buff_gy;
  *mean_gz = (int16_t)buff_gz;
}

void getOffset(int16_t* mean_gyrox, int16_t *mean_gyroy, int16_t *mean_gyroz){
  int gx_offset, gy_offset, gz_offset, giro_deadzone = 2;
  int16_t mean_gx, mean_gy, mean_gz;
  gx_offset = -*mean_gyrox / 4;
  gy_offset = -*mean_gyroy / 4;
  gz_offset = -*mean_gyroz / 4;
  while (1){
    int ready = 0;
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);
    getMeanRot(&mean_gx, &mean_gy, &mean_gz, 100);
    if (abs(mean_gx) <= giro_deadzone) ready++;
    else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

    if (abs(mean_gy) <= giro_deadzone) ready++;
    else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

    if (abs(mean_gz) <= giro_deadzone) ready++;
    else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);
    Serial.print(".");
    if (ready == 3) break;
  }
  *mean_gyrox = gx_offset;
  *mean_gyroy = gy_offset;
  *mean_gyroz = gz_offset;
}
