// Copyright: Mohammad Safeea, 2021-April-01
// ---------------------------------------
// ESP32 MPU QUAT 2 Serial program,
// It is used to:
// 1- Read data from MPU6050 over I2C,
// 2- Calculates quaternion using Mahony filter
// 3- It streams quaternion + gyro measurments over Serial.

// Tests:
// ------
// Tested successfully on {ESP32 Dev Module}

// Connections:
// ===============
// Connect GPIO22 of ESP32 to SCL of MPU6050
// Connect GPIO21 of ESP32 to SDA of MPU6050

// IN A NUTSHELL:
// ===============
// This program receives MPU6050 data over I2C, then it stream it over Serial

// Change gain:
// ===========
// you can change the Kp gain over serial by sending:
// "Kp0.05{chr(10)}"
// you can change the Ki gain over serial by sending:
// "Ki.05{chr(10)}"
// To stop update quaternion update set Kp and Ki to zero

//-- Libraries Included --------------------------------------------------------------
#include <Wire.h>
//====================================================================================
// MPU Variables
#define MPUaddress (0b1101000)
#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.0f) // 2 * integral gain
//---------------------------------------------------------------------------------------------------
// Mahony Variable definitions
float twoKp = twoKpDef;                      // 2 * proportional gain (Kp)
float twoKi = twoKiDef;                      // 2 * integral gain (Ki)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;          // quaternion of sensor frame relative to auxiliary frame
float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki
constexpr float INV_GYRO_SCALE = 1.0f / 131.0f;
constexpr float DEG2RAD        = 3.14159265f / 180.0f;

// Bytes buffer message
const unsigned int header_len=4;
const unsigned int terminator_len=4;
const unsigned int bytesMessageLen=7*4 + header_len + terminator_len;
byte header_bytes[header_len] = {0xFF, 0xFF, 0xFF, 0xFF};
byte terminator_bytes[terminator_len] = {0xEE, 0xEE, 0xEE, 0xEE};
byte message_bytes[bytesMessageLen];
//---------------------------------------------------------------------------------------------------
// Measurments
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ; // G force

long gyroX, gyroY, gyroZ; // degree per sec
long gyroX_bias, gyroY_bias, gyroZ_bias;
long gyroXCalibrated, gyroYCalibrated, gyroZCalibrated;
float rotX, rotY, rotZ; // [Degree/Second] after bias compensation
float wx, wy, wz; // [Rad/Sec] after bias compensation
//====================================================================================
// Gyro bias variables
long t0=0;
long counter =0;
float temp;
int cycle_count = 0;
//====================================================================================
// dt Variables
boolean firstExecution =  true;
unsigned long tPrevious;
unsigned long tNow;
float dt;
//====================================================================================
// SDA, SCL GPIO pins for the ESP32
#define I2C_SDA 21
#define I2C_SCL 22
// I2C request timeout (ms)
#define I2C_REQUEST_TIMEOUT_MS 10
//====================================================================================
#define LED_INDICATOR 2
long long led_time_millis;
//====================================================================================    
void setup() {
    // populte message with header and terminator
    for (int i=0;i<bytesMessageLen;i++)
    {
      if(i<4)
      {
        message_bytes[i]=header_bytes[i];
      }
      else if(i>=(bytesMessageLen - terminator_len))
      {
        message_bytes[i]=terminator_bytes[i - (bytesMessageLen - terminator_len)];
      }
    }
    // Set up serial
    Serial.begin(115200);
    // Setup MPU
    Wire.begin(I2C_SDA,I2C_SCL,100000);
    setupMPU();
    delay(500);
    
    t0=millis();
    gyroX_bias=0;gyroY_bias=0;gyroZ_bias=0;
    // calculate the bias in the gyro measurments
    delay(1000);
    calibrateGyroForBias();
    pinMode(LED_INDICATOR, OUTPUT);
    led_time_millis = millis();
    Serial.println("Starting main loop:");
}

void calibrateGyroForBias()
{
  long gyroX_sum,gyroY_sum,gyroZ_sum;
  gyroX_sum=0;gyroY_sum=0;gyroZ_sum=0;
  long daCount=0;
  long tStart=millis();
  while((millis()-tStart)<1000)
  {
    // use safe non-blocking reads with timeout; if a read fails, skip this sample
    bool ok1 = recordAccelRegisters();
    bool ok2 = recordGyroRegisters();
    if (ok1 && ok2) {
      daCount++;
      gyroX_sum=gyroX_sum+gyroX;
      gyroY_sum=gyroY_sum+gyroY;
      gyroZ_sum=gyroZ_sum+gyroZ;
    }
    // small pause to avoid hammering the bus
    delay(5);
  }
  if (daCount > 0) {
    gyroX_bias=gyroX_sum/daCount;
    gyroY_bias=gyroY_sum/daCount;
    gyroZ_bias=gyroZ_sum/daCount;
  } else {
    // No valid samples collected
    gyroX_bias = 0;
    gyroY_bias = 0;
    gyroZ_bias = 0;
    Serial.println("Warning: No gyro samples collected during calibration; biases set to 0");
  }
  Serial.print("gyroX_bias:  ");
  Serial.println(gyroX_bias);
  Serial.print("gyroY_bias:  ");
  Serial.println(gyroY_bias);
  Serial.print("gyroZ_bias:  ");
  Serial.println(gyroZ_bias);
}

void calibrateGyroBias()
{
  gyroXCalibrated = gyroX - gyroX_bias;
  gyroYCalibrated = gyroY - gyroY_bias;
  gyroZCalibrated = gyroZ - gyroZ_bias;
}
//====================================================================================
void dispAuthorsMessage()
{
  Serial.println("");
  Serial.println("Copyright: Mohammad Safeea, 2020-Dec-04");
  Serial.println("This is a program used to relay data received over I2C into Serial");
  Serial.println("....................");
}
//====================================================================================

void setupMPU(){
  Wire.beginTransmission(MPUaddress); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(MPUaddress); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(MPUaddress); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

//====================================================================================
static class ReadConstantsFromSerial {
private:
  float kp_buffer = 0.0f;
  float ki_buffer = 0.0f;
  char previous_char = '\0';
  float factor = 0.0f;
public:
  void update() {
    while (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'K') {
        previous_char = c;
        factor = 0.0f;
      } else if (previous_char == 'K' && c == 'p') {
        kp_buffer = 0.0f;
        previous_char = c;
      } else if (previous_char == 'K' && c == 'i') {
        ki_buffer = 0.0f;
        previous_char = c;
      } else if ((c >= '0') && (c <= '9')) {
        if (previous_char == 'p') {
          kp_buffer = kp_buffer + float(c - '0') * factor;
          factor = factor * 0.1f;
        } else if (previous_char == 'i') {
          ki_buffer = ki_buffer + float(c - '0') * factor;
          factor = factor * 0.1f;
        }
      } else if (c == '.') {
        factor = 0.1f;
      } else if (c == char(10)) { // terminator
        // end of command, apply values
        if (previous_char == 'p') {
          twoKp = 2.0f * kp_buffer;
        } else if (previous_char == 'i') {
          twoKi = 2.0f * ki_buffer;
        }
        previous_char = '\0';
      } else {
        // unexpected character, reset state
        previous_char = '\0';
      }
    }
  }
} params_reader;

//====================================================================================
  
void loop() {
   while(1)
   {
      long long current_time = millis();
      if (current_time - led_time_millis > 500){
        int value = 1 - digitalRead(LED_INDICATOR);
        digitalWrite(LED_INDICATOR, value);
        led_time_millis = current_time;
      }
      params_reader.update();
      bool ok1 = recordAccelRegisters();
      bool ok2 = recordGyroRegisters();
      if (ok1 && ok2) {
        calibrateGyroBias();
        processGyroData();
        calculate_dt();
        MahonyAHRSupdateIMU(wx, wy, wz, gForceX, gForceY, gForceZ);
        // stream data over serial
        for (int i = 0; i < 4; i++) {
          printDataAscii();
        }
        delay(2);
      }
   }
}

void printQuatAscii()
{
  Serial.print("Quat:");
  Serial.print(q0);
  Serial.print("_");
  Serial.print(q1);
  Serial.print("_");
  Serial.print(q2);
  Serial.print("_");
  Serial.println(q3);
}

void printGyroAscii()
{
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.println(rotZ);
}

void printAccelAscii()
{
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);
}

void streamBinaryOverSerial()
{
  int byteCount = 4; // after header
  byteCount = serializeData2Bytes(byteCount,q0);
  byteCount = serializeData2Bytes(byteCount,q1);
  byteCount = serializeData2Bytes(byteCount,q2);
  byteCount = serializeData2Bytes(byteCount,q3);
  byteCount = serializeData2Bytes(byteCount,rotX);
  byteCount = serializeData2Bytes(byteCount,rotY);
  byteCount = serializeData2Bytes(byteCount,rotZ);
  byteCount = serializeData2Bytes(byteCount,gForceX);
  byteCount = serializeData2Bytes(byteCount,gForceY);
  byteCount = serializeData2Bytes(byteCount,gForceZ);
  
  Serial.write(message_bytes, bytesMessageLen);
}


//====================================================================================
int serializeData2Bytes(int index,float x)
{
  byte* pointer;
  pointer = (byte*) &x;
  for(int i=3;i>-1;i--)
  {
    message_bytes[index] = pointer[i];
    index = index + 1;
  }
  return index;
}
//====================================================================================
bool recordAccelRegisters() {
  Wire.beginTransmission(MPUaddress); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(MPUaddress,6); //Request Accel Registers (3B - 40)
  unsigned long start = millis();
  while (Wire.available() < 6) {
    if ((millis() - start) > I2C_REQUEST_TIMEOUT_MS) {
      // timeout - no data
      Serial.println("Warning: accel read timeout");
      // consume any available bytes
      while (Wire.available()) Wire.read();
      return false;
    }
    delay(1);
  }
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelX=checkOverFlow(accelX);
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelY=checkOverFlow(accelY);
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  accelZ=checkOverFlow(accelZ);
  processAccelData();
  return true;
}

void processAccelData(){
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
}

bool recordGyroRegisters() {
  Wire.beginTransmission(MPUaddress); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(MPUaddress,6); //Request Gyro Registers (43 - 48)
  unsigned long start = millis();
  while (Wire.available() < 6) {
    if ((millis() - start) > I2C_REQUEST_TIMEOUT_MS) {
      Serial.println("Warning: gyro read timeout");
      while (Wire.available()) Wire.read();
      return false;
    }
    delay(1);
  }
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroX=checkOverFlow(gyroX);
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroY=checkOverFlow(gyroY);
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  gyroZ=checkOverFlow(gyroZ);
  return true;
}

long checkOverFlow(long x)
{
  long y;
  if(x>32767)
  {
    y=x-65536;
  }
  else
  {
    y=x;
  }
  return y;
}

void processGyroData() {
  rotX = gyroXCalibrated * INV_GYRO_SCALE;
  rotY = gyroYCalibrated * INV_GYRO_SCALE; 
  rotZ = gyroZCalibrated * INV_GYRO_SCALE;
  wx = rotX * DEG2RAD;
  wy = rotY * DEG2RAD;
  wz = rotZ * DEG2RAD;
}

void printDataAscii() {
  cycle_count = cycle_count + 1;
  int measruement;
  constexpr int factor = 1000;
  switch (cycle_count)
  {
    case 1:
      Serial.print(" a"); // qw
      measruement = q0*1000;
      Serial.print(measruement);
      break;
    case 2:
      Serial.print(" b"); // qx
      measruement = q1*1000;
      Serial.print(measruement);
      break;
    case 3:
      Serial.print(" c"); // qy
      measruement = q2*1000;
      Serial.print(measruement);
      break;
    case 4:
      Serial.print(" d"); // qz
      measruement = q3*1000;
      Serial.print(measruement);
      break;
    case 5:
      Serial.print(" e");
      measruement = wx*1000;
      Serial.print(measruement);
      break;
    case 6:
      Serial.print(" f");
      measruement = wy*1000;
      Serial.print(measruement);
      break;
    case 7:
      Serial.print(" g");
      measruement = wz*1000;
      Serial.print(measruement);
      break;
    case 8:
      Serial.print(" h");
      measruement = gForceX*1000;
      Serial.print(measruement);
      break;
    case 9:
      Serial.print(" i");
      measruement = gForceY*1000;
      Serial.print(measruement);
      break;
    case 10:
      Serial.print(" j");
      measruement = gForceZ*1000;
      Serial.print(measruement);
      Serial.println(" ");
      break;
  }
  cycle_count = cycle_count % 10;
}

//====================================================================================
// Calculate dt
float calculate_dt()
{
  if(firstExecution==true)
  {
    tPrevious=micros();
    tNow=tPrevious;
    firstExecution=false;
  }
  else
  {
    tNow=micros();
  }
  dt= (tNow-tPrevious)/1000000.0;
  tPrevious=tNow;
  return dt;
}
//====================================================================================
// Filter
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  if ((twoKp == 0.0) && (twoKi == 0.0)) return;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    temp=ax * ax + ay * ay + az * az;
    recipNorm = invSqrt(temp);
    ax = ax * recipNorm;
    ay = ay * recipNorm;
    az = az * recipNorm;        

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
  
    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx = integralFBx + twoKi * halfex * (dt);  // integral error scaled by Ki
      integralFBy = integralFBy + twoKi * halfey * (dt);
      integralFBz = integralFBz + twoKi * halfez * (dt);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx = gx + twoKp * halfex;
    gy = gy + twoKp * halfey;
    gz = gz + twoKp * halfez;
  }
  
  // Integrate rate of change of quaternion
  gx =  gx * (0.5f * dt);   // pre-multiply common factors
  gy = gy * (0.5f * dt);
  gz = gz * (0.5f * dt);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 = q0 + (-qb * gx - qc * gy - q3 * gz);
  q1 = q1 + (qa * gx + qc * gz - q3 * gy);
  q2 = q2 + (qa * gy - qb * gz + q3 * gx);
  q3 = q3 + (qa * gz + qb * gy - qc * gx); 
  
  // Normalise quaternion
  temp=q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
  recipNorm = invSqrt(temp);
  q0 = q0 * recipNorm;
  q1 = q1 * recipNorm;
  q2 = q2 * recipNorm;
  q3 = q3 * recipNorm;
}


float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}
