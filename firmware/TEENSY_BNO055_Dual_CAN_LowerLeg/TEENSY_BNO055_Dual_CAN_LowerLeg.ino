#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include <FlexCAN.h>

#define CUR_SENSOR_ID 55
#define CUR_SENSOR_ADDR 0x28
#define NUM_AXIS 4  // 3 + 1 see in the main() what's the 1
#define NUM_QUANT_AXIS 4
#define SAMPLE_RATE 100 // integer

#define CAN_ID_IMU1_ACCL 0x268
#define CAN_ID_IMU1_GYRO 0x269
#define CAN_ID_IMU1_OREN 0x26A
#define CAN_ID_IMU2_ACCL 0x26B
#define CAN_ID_IMU2_GYRO 0x26C
#define CAN_ID_IMU2_OREN 0x26D


#define INT16_MAX 32767
#define ACCL_RANGE 100
#define GYRO_RANGE 2000
#define OREN_ROLL 180
#define OREN_PITCH 90
#define OREN_YAW 360
#define QUANT_RANGE 1

uint32_t CANrate = 1000000;
FlexCAN CANbus(CANrate);
static CAN_message_t msgCAN;

bool zero = true;
bool calibrate = true;
int zeroTime = 50;
int numIMU = 2;

// Loop time
int curTime = 0;  // current time
int elpsTime = 0;  // time elasped in the current loop
int loopTime = 1000 / SAMPLE_RATE;  // loop time (ms)


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
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
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
void restore_cur_sensor(int id,const Adafruit_BNO055 &curIMU, int curEEAddress)
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

  /*Start Restore*/
  if (bnoID != sensor.sensor_id)
  {
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
  }
  else
  {
    /* Restore Calibration */
    curEEAddress += sizeof(long);
    adafruit_bno055_offsets_t calibrationData;
    EEPROM.get(curEEAddress, calibrationData);
    displaySensorOffsets(calibrationData);
    Serial.print("\n\nRestoring Calibration data to the BNO055 "); Serial.println(id); 
    curIMU.setSensorOffsets(calibrationData);
    Serial.print("\n\nCalibration data loaded into BNO055 "); Serial.println(id); 
    curIMU.setExtCrystalUse(true);
  }

  curIMU.setMode(OPERATION_MODE_NDOF);
  Serial.println("====================");
}

void toEulerAngles(double w, double x, double y, double z, double eularAngles[]) {
    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 2 * (w * w + z * z) - 1;
    eularAngles[0] = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (x * z - w * y);
    eularAngles[1] = -std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 2 * (w * w + x * x) - 1;
    eularAngles[2] = std::atan2(siny_cosp, cosy_cosp);
}

/**************************************************************************/
/*
    Return IMU data
    */
/**************************************************************************/
void sample_cur_sensor(int id,const Adafruit_BNO055 &curIMU)
{
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  curIMU.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  curIMU.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  // curIMU.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  // curIMU.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  curIMU.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // curIMU.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Quaternion quat = curIMU.getQuat();
 
  printEvent(&accelerometerData);
  printEvent(&angVelocityData);
  printEvent(&orientationData);

  double quat_euler[3] = {0};
  toEulerAngles(quat.w(), quat.x(), quat.y(), quat.z(), quat_euler);
  Serial.print(quat_euler[0]*180./3.14);
  Serial.print(", ");
  Serial.print(quat_euler[1]*180./3.14);
  Serial.print(", ");
  Serial.print(quat_euler[2]*180./3.14);
  Serial.println(" ");
  // printEvent(&linearAccelData);
  // printEvent(&magnetometerData);
  // printEvent(&gravityData);
}

void splitUint16(int16_t input, int8_t *high, int8_t *low) {
    *low = (int8_t)(input & 0xFF);           // Extract lower 8 bits
    *high = (int8_t)((input >> 8) & 0xFF);   // Extract higher 8 bits
}

void send_can_message(int imu_id, int16_t accl_data[], int16_t gyro_data[], int16_t oren_data[])
{
  uint32_t can_msg_id_accl, can_msg_id_gyro, can_msg_id_oren;
  int8_t msg_lower_8bit, msg_higher_8bit;
  int result;
  if (imu_id == 0){
    can_msg_id_accl = CAN_ID_IMU1_ACCL;
    can_msg_id_gyro = CAN_ID_IMU1_GYRO;
    can_msg_id_oren = CAN_ID_IMU1_OREN;
  }
  else if (imu_id == 1) {
    can_msg_id_accl = CAN_ID_IMU2_ACCL;
    can_msg_id_gyro = CAN_ID_IMU2_GYRO;
    can_msg_id_oren = CAN_ID_IMU2_OREN;
  }

  // Accel: 3-axis accelerometer (with gravity) + x-axis linear accelerometer (without gravity)
  for (int i = 0; i < NUM_AXIS; i++) {
    splitUint16(accl_data[i], &msg_higher_8bit, &msg_lower_8bit);
    msgCAN.buf[2*i] = msg_higher_8bit; msgCAN.buf[2*i+1] = msg_lower_8bit;
  }
  msgCAN.id = can_msg_id_accl;
  result = CANbus.write(msgCAN);

  // Gyro
  for (int i = 0; i < NUM_AXIS; i++) {
    splitUint16(gyro_data[i], &msg_higher_8bit, &msg_lower_8bit);
    msgCAN.buf[2*i] = msg_higher_8bit; msgCAN.buf[2*i+1] = msg_lower_8bit;
  }
  msgCAN.id = can_msg_id_gyro;
  result = CANbus.write(msgCAN);

  // Orientation
  for (int i = 0; i < NUM_QUANT_AXIS; i++) {
    splitUint16(oren_data[i], &msg_higher_8bit, &msg_lower_8bit);
    msgCAN.buf[2*i] = msg_higher_8bit; msgCAN.buf[2*i+1] = msg_lower_8bit;
  }
  msgCAN.id = can_msg_id_oren;
  result = CANbus.write(msgCAN);
}

int range_mapping(float sensor_data, float sensor_range, int msg_max)
{
    int capped_data = min(max(sensor_data, -sensor_range), sensor_range);
    int converted_data = (int)((sensor_data / sensor_range) * msg_max);
    converted_data = min(max(converted_data, -INT16_MAX), INT16_MAX);
    return converted_data;
}
 
void setup(void)
{
  CANbus.begin();

  delay(1000);

  msgCAN.ext = 0;
  msgCAN.len = 8;
  msgCAN.timeout = 1;
  msgCAN.buf[0] = 0; msgCAN.buf[1] = 0; msgCAN.buf[2] = 0; msgCAN.buf[3] = 0;
  msgCAN.buf[4] = 0; msgCAN.buf[5] = 0; msgCAN.buf[6] = 0; msgCAN.buf[7] = 0;

  Serial.begin(115200);
  delay(500);
  for (int id = 0; id < numIMU; id++)
  {
    Serial.print("BNO055 "); Serial.print(id); Serial.println("");
    if (id == 0)
    {
      restore_cur_sensor(id,bno,eeAddress);
    }
    else if (id == 1)
    {
      restore_cur_sensor(id,bno2,eeAddress2);
    }
  }
  delay(1000);
}

sensors_event_t orientationEvent, angVelocityEvent, accelerometerEvent, linearAccelEvent;
double linAcc_x = 0.0, linAcc_y = 0.0, linAcc_z = 0.0, linAcc_vecMag = 0.0;  // combined 3-axis acceleration without gravity
int16_t msg_accl[4] = {0}, msg_gyro[4] = {0}, msg_oren[4] = {0};

void loop(void)
{
  // Get current time
  curTime = millis();

  for (int id = 0; id < numIMU; id++)
  {
    //Serial.print("BNO055 "); Serial.print(id); Serial.println(" test");
    if (id == 0)
    {
      //sample_cur_sensor(id,bno);
      
      
      
      bno.getEvent(&accelerometerEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
      bno.getEvent(&angVelocityEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
      bno.getEvent(&linearAccelEvent, Adafruit_BNO055::VECTOR_LINEARACCEL);
      //bno.getEvent(&orientationEvent, Adafruit_BNO055::VECTOR_EULER);

      msg_accl[0] = range_mapping(accelerometerEvent.acceleration.x, ACCL_RANGE, INT16_MAX);
      msg_accl[1] = range_mapping(accelerometerEvent.acceleration.y, ACCL_RANGE, INT16_MAX);
      msg_accl[2] = range_mapping(accelerometerEvent.acceleration.z, ACCL_RANGE, INT16_MAX);
      msg_accl[3] = range_mapping(linAcc_x, ACCL_RANGE, INT16_MAX);

      msg_gyro[0] = range_mapping(angVelocityEvent.gyro.x, GYRO_RANGE, INT16_MAX);
      msg_gyro[1] = range_mapping(angVelocityEvent.gyro.y, GYRO_RANGE, INT16_MAX);
      msg_gyro[2] = range_mapping(angVelocityEvent.gyro.z, GYRO_RANGE, INT16_MAX);
      msg_gyro[3] = range_mapping(linAcc_y, ACCL_RANGE, INT16_MAX);

      imu::Quaternion quat = bno.getQuat();
      msg_oren[0] = range_mapping(quat.w(), QUANT_RANGE, INT16_MAX);  // yaw
      msg_oren[1] = range_mapping(quat.x(), QUANT_RANGE, INT16_MAX);  //pitch
      msg_oren[2] = range_mapping(quat.y(), QUANT_RANGE, INT16_MAX);  // roll
      msg_oren[3] = range_mapping(quat.z(), QUANT_RANGE, INT16_MAX);  // roll

      send_can_message(id, msg_accl, msg_gyro, msg_oren);
      
      

      //displaySensorDetails(bno);

    }
    else if (id == 1)
    {
      //sample_cur_sensor(id,bno2);
      //Serial.println("=========");
      
      
      bno2.getEvent(&accelerometerEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
      bno2.getEvent(&angVelocityEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
      bno2.getEvent(&linearAccelEvent, Adafruit_BNO055::VECTOR_LINEARACCEL);
      //bno2.getEvent(&orientationEvent, Adafruit_BNO055::VECTOR_EULER);

      linAcc_x = linearAccelEvent.acceleration.x;
      linAcc_y = linearAccelEvent.acceleration.y;
      linAcc_z = linearAccelEvent.acceleration.z;
      linAcc_vecMag = sqrt(pow(linAcc_x, 2) + pow(linAcc_y, 2) + pow(linAcc_z, 2));

      msg_accl[0] = range_mapping(accelerometerEvent.acceleration.x, ACCL_RANGE, INT16_MAX);
      msg_accl[1] = range_mapping(accelerometerEvent.acceleration.y, ACCL_RANGE, INT16_MAX);
      msg_accl[2] = range_mapping(accelerometerEvent.acceleration.z, ACCL_RANGE, INT16_MAX);
      msg_accl[3] = range_mapping(linAcc_z, ACCL_RANGE, INT16_MAX);

      msg_gyro[0] = range_mapping(angVelocityEvent.gyro.x, GYRO_RANGE, INT16_MAX);
      msg_gyro[1] = range_mapping(angVelocityEvent.gyro.y, GYRO_RANGE, INT16_MAX);
      msg_gyro[2] = range_mapping(angVelocityEvent.gyro.z, GYRO_RANGE, INT16_MAX);
      msg_gyro[3] = range_mapping(linAcc_vecMag, ACCL_RANGE, INT16_MAX);

      imu::Quaternion quat = bno2.getQuat();
      msg_oren[0] = range_mapping(quat.w(), QUANT_RANGE, INT16_MAX);  // yaw
      msg_oren[1] = range_mapping(quat.x(), QUANT_RANGE, INT16_MAX);  //pitch
      msg_oren[2] = range_mapping(quat.y(), QUANT_RANGE, INT16_MAX);  // roll
      msg_oren[3] = range_mapping(quat.z(), QUANT_RANGE, INT16_MAX);  // roll
      
      send_can_message(id, msg_accl, msg_gyro, msg_oren);
      
      
    }
  }

  
 
  //Serial.println("--");
  //delay(BNO055_SAMPLERATE_DELAY_MS);

  // Delay for targeted loop time
  elpsTime = millis() - curTime;
  while (elpsTime < loopTime)
  {
    elpsTime = millis() - curTime;
  }
}
 
void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }
 
  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}