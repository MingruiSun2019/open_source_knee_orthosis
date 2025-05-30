// This example is for sending data to CAN bus with two BNO055 IMUs connected to Teensy 4.1
// It is intented to be an example of firmware that work with the CANIMU module in CORC
// Check the CORC code here: https://github.com/UniMelbHumanRoboticsLab/CANOpenRobotController
//
// Author: Mingrui Sun
// Date: 2025-05-30

// The idea is:
// The IMU data is sent to the MCU through I2C
// The MCU quantises the data (16-bit integer per axis) and package them into CAN messages
// The MCU sends the CAN messages to the CAN bus

// For the two IMUs, the data is sent in the following format:
// - CAN message 1 (8 bytes): 6 bytes of acceleration data of the 1st IMU + 2 bytes of zeros
// - CAN message 2 (8 bytes): 6 bytes of linear acceleration data of the 1st IMU + 2 bytes of zeros
// - CAN message 3 (8 bytes): 8 bytes of quaternion data of the 1st IMU
// - CAN message 4 (8 bytes): 6 bytes of acceleration data of the 2nd IMU + 2 bytes of zeros
// - CAN message 5 (8 bytes): 6 bytes of linear acceleration data of the 2nd IMU + 2 bytes of zeros
// - CAN message 6 (8 bytes): 8 bytes of quaternion data of the 2nd IMU


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include <FlexCAN.h>

#define SENSOR_ID 55  // This is some parameters of I2C device address of the BNO055
#define SENSOR_ADDR 0x28  // This is some parameters of I2C device address of the BNO055
#define NUM_INT16_PER_CAN_MSG 4 // 4 int16_t data per CAN message
#define SAMPLE_RATE 100 // Hz

#define CAN_ID_IMU1_ACCL 0x272
#define CAN_ID_IMU1_LINACCL 0x273
#define CAN_ID_IMU1_OREN 0x274
#define CAN_ID_IMU2_ACCL 0x275
#define CAN_ID_IMU2_LINACCL 0x276
#define CAN_ID_IMU2_OREN 0x277

// IMU parameters
// Make sure these parameters match the ones in CORC
#define INT16_MAX 32767
#define ACCL_RANGE 100  // m/s^2, apply for both acceleration and linear acceleration
#define QUANT_RANGE 1  // unitless

// CAN bus settings
// Make sure the CAN rate matches the one in CORC
uint32_t canRate = 1000000;
FlexCAN canBus(canRate);
static CAN_message_t msgCAN;

// Loop time
int curTime = 0;  // current time
int elpsTime = 0;  // time elasped in the current loop
int loopTime = 1000 / SAMPLE_RATE;  // loop time (ms)
 
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
Adafruit_BNO055 bno1 = Adafruit_BNO055(SENSOR_ID, SENSOR_ADDR);
Adafruit_BNO055 bno2 = Adafruit_BNO055(SENSOR_ID+1, SENSOR_ADDR+1);
int eeAddress = 0;
int eeAddress2 = sizeof(long)+sizeof(adafruit_bno055_offsets_t);

/**************************************************************************/
/*
    Retrieve the stored calibration data to the IMU
    */
/**************************************************************************/
void restoreCurrentSensor(int imuID, const Adafruit_BNO055 &curIMU, int curEEAddress)
{
  // Initialise the sensor
  if (!curIMU.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections
    Serial.print("Ooops, BNO055 "); Serial.print(imuID); Serial.println(" not detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  Serial.print("BNO055 "); Serial.print(imuID); Serial.print(" found with EEPROM Address "); Serial.println(curEEAddress);

  int32_t bnoID = imuID;
  sensor_t sensor;
  curIMU.getSensor(&sensor);
  EEPROM.get(curEEAddress, bnoID);

  // Start Restore
  if (bnoID != sensor.sensor_id)
  {
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
  }
  else
  {
    // Restore Calibration
    curEEAddress += sizeof(long);
    adafruit_bno055_offsets_t calibrationData;
    EEPROM.get(curEEAddress, calibrationData);
    Serial.print("\n\nRestoring Calibration data to the BNO055 "); Serial.println(imuID); 
    curIMU.setSensorOffsets(calibrationData);
    Serial.print("\n\nCalibration data loaded into BNO055 "); Serial.println(imuID); 
    curIMU.setExtCrystalUse(true);
  }

  curIMU.setMode(OPERATION_MODE_NDOF);
  Serial.println("====================");
}


/**************************************************************************/
/*
    Split 16-bit integer into two 8-bit integers
    */
/**************************************************************************/
void splitUint16(int16_t input, int8_t *high, int8_t *low) {
    *low = (int8_t)(input & 0xFF);           // Extract lower 8 bits
    *high = (int8_t)((input >> 8) & 0xFF);   // Extract higher 8 bits
}


/**************************************************************************/
/*
    Send CAN message
    - 3 CAN messages are sent:
        - Acceleration: takes 6 bytes, 2 bytes for each axis
        - Linear Acceleration: takes 6 bytes, 2 bytes for each axis
        - Quaternion: takes 8 bytes, 2 bytes for each axis
    */
/**************************************************************************/
void sendCanMessage(int imuId, int16_t accData[], int16_t linAccData[], int16_t quatData[])
{
  uint32_t canMsgIdAccl, canMsgIdLinAcc, canMsgIdOren;
  int8_t msgLower8bit, msgHigher8bit;
  int result;

  // Set the CAN message ID based on the IMU ID
  if (imuId == 0){
    canMsgIdAccl = CAN_ID_IMU1_ACCL;
    canMsgIdLinAcc = CAN_ID_IMU1_LINACCL;
    canMsgIdOren = CAN_ID_IMU1_OREN;
  }
  else if (imuId == 1) {
    canMsgIdAccl = CAN_ID_IMU2_ACCL;
    canMsgIdLinAcc = CAN_ID_IMU2_LINACCL;
    canMsgIdOren = CAN_ID_IMU2_OREN;
  }

  // Acceleration
  for (int i = 0; i < NUM_INT16_PER_CAN_MSG; i++) {
    splitUint16(accData[i], &msgHigher8bit, &msgLower8bit);
    msgCAN.buf[2*i] = msgHigher8bit; msgCAN.buf[2*i+1] = msgLower8bit;
  }
  msgCAN.id = canMsgIdAccl;
  result = canBus.write(msgCAN);

  // Linear Acceleration
  for (int i = 0; i < NUM_INT16_PER_CAN_MSG; i++) {
    splitUint16(linAccData[i], &msgHigher8bit, &msgLower8bit);
    msgCAN.buf[2*i] = msgHigher8bit; msgCAN.buf[2*i+1] = msgLower8bit;
  }
  msgCAN.id = canMsgIdLinAcc;
  result = canBus.write(msgCAN);

  // Quaternion
  for (int i = 0; i < NUM_INT16_PER_CAN_MSG; i++) {
    splitUint16(quatData[i], &msgHigher8bit, &msgLower8bit);
    msgCAN.buf[2*i] = msgHigher8bit; msgCAN.buf[2*i+1] = msgLower8bit;
  }
  msgCAN.id = canMsgIdOren;
  result = canBus.write(msgCAN);
}

int rangeMapping(float sensorData, float sensorRange, int msgMax)
{
    float cappedData = min(max(sensorData, -sensorRange), sensorRange);
    int convertedData = (int)((cappedData / sensorRange) * msgMax);
    return min(max(convertedData, -INT16_MAX), INT16_MAX);
}
 
void setup(void)
{
  canBus.begin();

  delay(1000);

  msgCAN.ext = 0;
  msgCAN.len = 8;
  msgCAN.timeout = 1;
  msgCAN.buf[0] = 0; msgCAN.buf[1] = 0; msgCAN.buf[2] = 0; msgCAN.buf[3] = 0;
  msgCAN.buf[4] = 0; msgCAN.buf[5] = 0; msgCAN.buf[6] = 0; msgCAN.buf[7] = 0;

  Serial.begin(115200);
  delay(500);

  restoreCurrentSensor(0, bno1, eeAddress);
  restoreCurrentSensor(1, bno2, eeAddress2);

  delay(1000);
}

sensors_event_t accelerometerEvent, linearAccelEvent;
imu::Quaternion quat;
int16_t msgAcc[4] = {0}, msgLinAcc[4] = {0}, msgQuat[4] = {0};

void loop(void)
{
  // Get current time
  curTime = millis();

  // ============= IMU 1 =============
  bno1.getEvent(&accelerometerEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno1.getEvent(&linearAccelEvent, Adafruit_BNO055::VECTOR_LINEARACCEL);

  msgAcc[0] = rangeMapping(accelerometerEvent.acceleration.x, ACCL_RANGE, INT16_MAX);
  msgAcc[1] = rangeMapping(accelerometerEvent.acceleration.y, ACCL_RANGE, INT16_MAX);
  msgAcc[2] = rangeMapping(accelerometerEvent.acceleration.z, ACCL_RANGE, INT16_MAX);
  msgAcc[3] = 0;

  msgLinAcc[0] = rangeMapping(linearAccelEvent.acceleration.x, ACCL_RANGE, INT16_MAX);
  msgLinAcc[1] = rangeMapping(linearAccelEvent.acceleration.y, ACCL_RANGE, INT16_MAX);
  msgLinAcc[2] = rangeMapping(linearAccelEvent.acceleration.z, ACCL_RANGE, INT16_MAX);
  msgLinAcc[3] = 0;

  quat = bno1.getQuat();
  msgQuat[0] = rangeMapping(quat.w(), QUANT_RANGE, INT16_MAX);
  msgQuat[1] = rangeMapping(quat.x(), QUANT_RANGE, INT16_MAX);
  msgQuat[2] = rangeMapping(quat.y(), QUANT_RANGE, INT16_MAX);
  msgQuat[3] = rangeMapping(quat.z(), QUANT_RANGE, INT16_MAX);

  sendCanMessage(0, msgAcc, msgLinAcc, msgQuat);

  // ============= IMU 2 =============
  bno2.getEvent(&accelerometerEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno2.getEvent(&linearAccelEvent, Adafruit_BNO055::VECTOR_LINEARACCEL);

  msgAcc[0] = rangeMapping(accelerometerEvent.acceleration.x, ACCL_RANGE, INT16_MAX);
  msgAcc[1] = rangeMapping(accelerometerEvent.acceleration.y, ACCL_RANGE, INT16_MAX);
  msgAcc[2] = rangeMapping(accelerometerEvent.acceleration.z, ACCL_RANGE, INT16_MAX);
  msgAcc[3] = 0;

  msgLinAcc[0] = rangeMapping(linearAccelEvent.acceleration.x, ACCL_RANGE, INT16_MAX);
  msgLinAcc[1] = rangeMapping(linearAccelEvent.acceleration.y, ACCL_RANGE, INT16_MAX);
  msgLinAcc[2] = rangeMapping(linearAccelEvent.acceleration.z, ACCL_RANGE, INT16_MAX);
  msgLinAcc[3] = 0;

  quat = bno2.getQuat();
  msgQuat[0] = rangeMapping(quat.w(), QUANT_RANGE, INT16_MAX);
  msgQuat[1] = rangeMapping(quat.x(), QUANT_RANGE, INT16_MAX);
  msgQuat[2] = rangeMapping(quat.y(), QUANT_RANGE, INT16_MAX);
  msgQuat[3] = rangeMapping(quat.z(), QUANT_RANGE, INT16_MAX);
  
  sendCanMessage(1, msgAcc, msgLinAcc, msgQuat);

  // Delay for targeted loop time
  elpsTime = millis() - curTime;
  while (elpsTime < loopTime)
  {
    elpsTime = millis() - curTime;
  }
}