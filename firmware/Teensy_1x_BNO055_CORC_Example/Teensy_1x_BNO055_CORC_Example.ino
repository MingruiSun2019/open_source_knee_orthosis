// This example is for a single BNO055 IMU with Teensy 4.1
// It is used to send the IMU data to CORC
// The IMU data is sent in the following format:
// - Acceleration
// - Linear Acceleration
// - Quaternion

// Connect the IMU to the Teensy 4.1
// - SCL to analog 5??
// - SDA to analog 4??
// - VDD to 3.3-5V DC
// - GROUND to common ground

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include <FlexCAN.h>

#define CUR_SENSOR_ID 55
#define CUR_SENSOR_ADDR 0x28
#define NUM_INT16_PER_CAN_MSG 4
#define SAMPLE_RATE 100 // integer

#define CAN_ID_IMU1_ACCL 0x282
#define CAN_ID_IMU1_LINACCL 0x283
#define CAN_ID_IMU1_OREN 0x284

// IMU parameters
// Make sure these parameters match the ones in CORC
#define INT16_MAX 32767
#define ACCL_RANGE 100
#define GYRO_RANGE 2000
#define QUANT_RANGE 1

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
Adafruit_BNO055 bno = Adafruit_BNO055(CUR_SENSOR_ID, CUR_SENSOR_ADDR);
int eeAddress = 0;

/**************************************************************************/
/*
    Retrieve the stored calibration data to the IMU
    */
/**************************************************************************/
void restoreCurrentSensor()
{
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, BNO055 not detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  Serial.print("BNO055 found with EEPROM Address "); Serial.println(eeAddress);

  int32_t bnoID;
  sensor_t sensor;
  bno.getSensor(&sensor);
  EEPROM.get(eeAddress, bnoID);

  /*Start Restore*/
  if (bnoID != sensor.sensor_id)
  {
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
  }
  else
  {
    /* Restore Calibration */
    eeAddress += sizeof(long);
    adafruit_bno055_offsets_t calibrationData;
    EEPROM.get(eeAddress, calibrationData);
    Serial.println("\n\nRestoring Calibration data to the BNO055"); 
    bno.setSensorOffsets(calibrationData);
    Serial.println("\n\nCalibration data loaded into BNO055"); 
    bno.setExtCrystalUse(true);
  }

  bno.setMode(OPERATION_MODE_NDOF);
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
void sendCanMessage(int16_t accData[], int16_t linAccData[], int16_t quatData[])
{
  int8_t msgLower8bit, msgHigher8bit;
  int result;

  // Acceleration
  for (int i = 0; i < NUM_INT16_PER_CAN_MSG; i++) {
    splitUint16(accData[i], &msgHigher8bit, &msgLower8bit);
    msgCAN.buf[2*i] = msgHigher8bit; msgCAN.buf[2*i+1] = msgLower8bit;
  }
  msgCAN.id = CAN_ID_IMU1_ACCL;
  result = canBus.write(msgCAN);

  // Linear Acceleration
  for (int i = 0; i < NUM_INT16_PER_CAN_MSG; i++) {
    splitUint16(linAccData[i], &msgHigher8bit, &msgLower8bit);
    msgCAN.buf[2*i] = msgHigher8bit; msgCAN.buf[2*i+1] = msgLower8bit;
  }
  msgCAN.id = CAN_ID_IMU1_LINACCL;
  result = canBus.write(msgCAN);

  // Quaternion
  for (int i = 0; i < NUM_INT16_PER_CAN_MSG; i++) {
    splitUint16(quatData[i], &msgHigher8bit, &msgLower8bit);
    msgCAN.buf[2*i] = msgHigher8bit; msgCAN.buf[2*i+1] = msgLower8bit;
  }
  msgCAN.id = CAN_ID_IMU1_OREN;
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

  restoreCurrentSensor();

  delay(1000);
}

sensors_event_t accelerometerEvent, linearAccelEvent;
imu::Quaternion quat;
int16_t msgAcc[4] = {0}, msgLinAcc[4] = {0}, msgQuat[4] = {0};

void loop(void)
{
  // Get current time
  curTime = millis();

  // Get IMU data
  bno.getEvent(&accelerometerEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&linearAccelEvent, Adafruit_BNO055::VECTOR_LINEARACCEL);

  msgAcc[0] = rangeMapping(accelerometerEvent.acceleration.x, ACCL_RANGE, INT16_MAX);
  msgAcc[1] = rangeMapping(accelerometerEvent.acceleration.y, ACCL_RANGE, INT16_MAX);
  msgAcc[2] = rangeMapping(accelerometerEvent.acceleration.z, ACCL_RANGE, INT16_MAX);
  msgAcc[3] = 0;

  msgLinAcc[0] = rangeMapping(linearAccelEvent.acceleration.x, ACCL_RANGE, INT16_MAX);
  msgLinAcc[1] = rangeMapping(linearAccelEvent.acceleration.y, ACCL_RANGE, INT16_MAX);
  msgLinAcc[2] = rangeMapping(linearAccelEvent.acceleration.z, ACCL_RANGE, INT16_MAX);
  msgLinAcc[3] = 0;

  quat = bno.getQuat();
  msgQuat[0] = rangeMapping(quat.w(), QUANT_RANGE, INT16_MAX);  // yaw
  msgQuat[1] = rangeMapping(quat.x(), QUANT_RANGE, INT16_MAX);  //pitch
  msgQuat[2] = rangeMapping(quat.y(), QUANT_RANGE, INT16_MAX);  // roll
  msgQuat[3] = rangeMapping(quat.z(), QUANT_RANGE, INT16_MAX);  // roll

  sendCanMessage(msgAcc, msgLinAcc, msgQuat);

  // Delay for targeted loop time
  elpsTime = millis() - curTime;
  while (elpsTime < loopTime)
  {
    elpsTime = millis() - curTime;
  }
}
