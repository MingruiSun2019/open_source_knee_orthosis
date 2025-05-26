#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#define CUR_SENSOR_ID 55
#define CUR_SENSOR_ADDR 0x28

bool zero = true;
bool calibrate = true;
int zeroTime = 50;
 
/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
 
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.
 
   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
 
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground
 
   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/
 
/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
uint16_t BNO055_CALIBRATE_DELAY_MS = 10;
 
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(CUR_SENSOR_ID, CUR_SENSOR_ADDR);
Adafruit_BNO055 bno2 = Adafruit_BNO055(CUR_SENSOR_ID+1, CUR_SENSOR_ADDR+1);
int eeAddress = 0;
int eeAddress2 = sizeof(long)+sizeof(adafruit_bno055_offsets_t);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
/**************************************************************************/
void displaySensorDetails(const Adafruit_BNO055 &curIMU)
{
    sensor_t sensor;
    curIMU.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displaySensorStatus(const Adafruit_BNO055 &curIMU)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    curIMU.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(const Adafruit_BNO055 &curIMU)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    curIMU.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

/**************************************************************************/
/*
    Perform calibration on current IMU
    */
/**************************************************************************/
void calibrate_cur_sensor(int id,const Adafruit_BNO055 &curIMU, int curEEAddress)
{
  /* Initialise the sensor */
  if (!curIMU.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, BNO055 "); Serial.print(id); Serial.println(" not detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  Serial.print("BNO055 "); Serial.print(id); Serial.print(" found with EEPROM Address "); Serial.println(curEEAddress);

  int32_t bnoID = 0;
  sensor_t sensor;
  curIMU.getSensor(&sensor);
  EEPROM.get(curEEAddress, bnoID);

  /* 
  *  Start calibration
  *  Look for the sensor's unique ID
  */
  bool foundCalib = false;
  if (bnoID != sensor.sensor_id)
  {
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
  }
  else
  {
      adafruit_bno055_offsets_t calibrationData;
      // preferences.getBytes("calibrationData",&calibrationData, sizeof(calibrationData));
      curEEAddress += sizeof(long);
      EEPROM.get(curEEAddress, calibrationData);
      curEEAddress -= sizeof(long);
      Serial.print("\nFound Calibration for BNO055"); Serial.print(id); Serial.println(" in EEPROM.");
      displaySensorOffsets(calibrationData);
      Serial.print("\n\nRestoring Calibration data to the BNO055 "); Serial.println(id);
      curIMU.setSensorOffsets(calibrationData);
      Serial.print("\n\nCalibration data loaded into BNO055 "); Serial.println(id);
      foundCalib = true;
  }

  delay(1000);
  /* Display some basic information on this sensor */
  displaySensorDetails(curIMU);
  //Crystal must be configured AFTER loading calibration data into BNO055.
  curIMU.setExtCrystalUse(true);

  if (foundCalib)
  {
      Serial.print("Recalibrating BNO055 "); Serial.println(id);
  }
  else
  {
      Serial.print("Calibrating BNO055 "); Serial.println(id);
  }

  while (!curIMU.isFullyCalibrated())
  {
      /* Optional: Display calibration status */
      displayCalStatus(curIMU);
      /* New line for the next sample */
      Serial.println("");
      /* Wait the specified delay before requesting new data */
      delay(BNO055_CALIBRATE_DELAY_MS);
  }

  Serial.print("\nBNO055 "); Serial.print(id); Serial.println(" Fully Calibrated.");
  Serial.println("--------------------------------");
  Serial.println("Calibration Results: ");
  adafruit_bno055_offsets_t newCalib;
  curIMU.getSensorOffsets(newCalib);
  displaySensorOffsets(newCalib);

  Serial.println("\n\nStoring new calibration data to EEPROM...");
  curIMU.getSensor(&sensor);
  bnoID = sensor.sensor_id;
  EEPROM.put(curEEAddress, bnoID);
  delay(1000);
  curEEAddress += sizeof(long);
  EEPROM.put(curEEAddress, newCalib);
  Serial.println("Data stored to EEPROM.");
  
  curIMU.setMode(OPERATION_MODE_NDOF);
  Serial.println("====================");
}
 
void setup(void)
{
  Serial.begin(115200);
  delay(500);
  for (int id = 0; id < 2; id++)
  {
    Serial.print("BNO055 "); Serial.print(id); Serial.println(" test"); Serial.println("");
    if (id == 0)
    {
      calibrate_cur_sensor(id,bno,eeAddress);
    }
    else if (id == 1)
    {
      calibrate_cur_sensor(id,bno2,eeAddress2);
    }
  }
  delay(1000);
}
 
void loop(void)
{
}