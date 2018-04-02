
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define I2C_ADDR 0x68
#define MPU_INTERRUPT 2

#define MPU_XG_OFFSET 92
#define MPU_YG_OFFSET 0
#define MPU_ZG_OFFSET 9
#define MPU_XA_OFFSET 113
#define MPU_YA_OFFSET -4223
#define MPU_ZA_OFFSET 1563

#define SERIAL_RATE 38400
#define START_DELAY 5

#define MAIN_ROTOR_BOTTOM 3
#define MAIN_ROTOR_TOP 6
#define TAIL_ROTOR 5

MPU6050 mpu(I2C_ADDR);
bool dmpReady = false;
uint8_t mpuIntStatus;
volatile bool mpuInterrupt = false;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion quaternion;
VectorInt16 accel;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;

int thrust = 100;
int yaw = 0;
int pitch = 0;

bool ledActivity = false;

void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  Wire.begin();
  Serial.begin(SERIAL_RATE);
  
  pinMode(MPU_INTERRUPT, INPUT);

  mpu.initialize();
  
  mpu.setXGyroOffset(MPU_XG_OFFSET);
  mpu.setXAccelOffset(MPU_XA_OFFSET);
  mpu.setYGyroOffset(MPU_YG_OFFSET);
  mpu.setYAccelOffset(MPU_YA_OFFSET);
  mpu.setZGyroOffset(MPU_ZG_OFFSET);
  mpu.setZAccelOffset(MPU_ZA_OFFSET);
  
  if (!mpu.testConnection()) {
    Serial.println("mpu.testConnection() failed");
    
    return;
  }
  
  uint8_t devStatus = mpu.dmpInitialize();

  if (devStatus != 0) {
    Serial.print("DMP Init. failed. (code ");
    Serial.print(devStatus);
    Serial.println(")");
    
    return;
  }
  
  mpu.setDMPEnabled(true);

  attachInterrupt(
    digitalPinToInterrupt(MPU_INTERRUPT), 
    dmpDataReady, 
    RISING
  );
  
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();
  dmpReady = true;
    
  Serial.println("Preflight checks passed!");

  for (uint8_t i = 0; i < START_DELAY; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(900);
  }
  
  for (uint8_t i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

void loop() {
  if (!dmpReady) {
    delay(1000);
    return;
  }

  while (!mpuInterrupt && fifoCount < packetSize) {
    analogWrite(MAIN_ROTOR_BOTTOM, 255 - thrust);
    analogWrite(MAIN_ROTOR_TOP, 255 - thrust);
    analogWrite(TAIL_ROTOR, 255 - pitch);
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
    mpu.dmpGetAccel(&accel, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &quaternion);
    mpu.dmpGetLinearAccel(&aaReal, &accel, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &quaternion);
    
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
  }
}
