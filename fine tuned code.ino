#define PRINT_DEBUG_BUILD  // This is to print the MPU data on serial monitor to debug

// PID library
#include <PID_v1.h>

// These are needed for MPU
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
#define INTERRUPT_PIN 2

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];
VectorInt16 gy;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

#define PID_MIN_LIMIT -80
#define PID_MAX_LIMIT 80
#define PID_SAMPLE_TIME_IN_MILLI 10

#define SETPOINT_PITCH_ANGLE_OFFSET -2.5
#define MIN_ABSOLUTE_SPEED 0

double setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET;
double pitchGyroAngle = 0;
double pitchPIDOutput = 0;

double setpointYawRate = 0;
double yawGyroRate = 0;
double yawPIDOutput = 0;

#define PID_PITCH_KP 60
#define PID_PITCH_KI 0.3 
#define PID_PITCH_KD 0.9

#define PID_YAW_KP 0.3
#define PID_YAW_KI 0.1
#define PID_YAW_KD 0.01

PID pitchPID(&pitchGyroAngle, &pitchPIDOutput, &setpointPitchAngle, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, DIRECT);
PID yawPID(&yawGyroRate, &yawPIDOutput, &setpointYawRate, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, DIRECT);

int enableMotor1 = 9;
int motor1Pin1 = 5;
int motor1Pin2 = 6;

int motor2Pin1 = 7;
int motor2Pin2 = 8;
int enableMotor2 = 10;

void setupPID()
{
  pitchPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);

  yawPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);
}

void setupMotors()
{
  pinMode(enableMotor1, OUTPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);

  pinMode(enableMotor2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  rotateMotor(0, 0);
}

void setupMPU()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(-984.00000); 
  mpu.setYAccelOffset(1217.00000); 
  mpu.setZAccelOffset(4916.00000);   
  mpu.setXGyroOffset(-213.00000);
  mpu.setYGyroOffset(17.00000);
  mpu.setZGyroOffset(-8.00000);  

  if (devStatus == 0) 
  {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else 
  {
    Serial.print("MPU initialization failed with code: ");
    Serial.println(devStatus);
  }
}

void setup()
{
  Serial.begin(115200);      // ✅ Enable serial monitor
  delay(2000);               // Optional: Give time for serial monitor to start
  Serial.println("Serial initialized...");

  setupMotors();
  setupMPU();
  setupPID();
}

void loop()
{
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gy, fifoBuffer);

    yawGyroRate = gy.z;
    pitchGyroAngle = ypr[1] * 180 / M_PI;
    pitchGyroAngle = pitchGyroAngle;

    pitchPID.Compute();
    yawPID.Compute();

    rotateMotor(pitchPIDOutput + yawPIDOutput, pitchPIDOutput - yawPIDOutput);

    #ifdef PRINT_DEBUG_BUILD
      Serial.println("----- DEBUG -----");
      Serial.print("Pitch Angle: ");
      Serial.println(pitchGyroAngle);
      Serial.print("Setpoint: ");
      Serial.println(setpointPitchAngle);
      Serial.print("PID Output: ");
      Serial.println(pitchPIDOutput);
      Serial.print("Yaw Rate: ");
      Serial.println(yawGyroRate);
      Serial.print("Yaw Output: ");
      Serial.println(yawPIDOutput);
      Serial.println("-----------------");
      delay(500);
    #endif
  }
}

void rotateMotor(int speed1, int speed2)
{
  if (speed1 < 0)
  {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);    
  }
  else
  {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);      
  }

  if (speed2 < 0)
  {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);    
  }
  else
  {
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);      
  }

  speed1 = abs(speed1) + MIN_ABSOLUTE_SPEED;
  speed2 = abs(speed2) + MIN_ABSOLUTE_SPEED;

  speed1 = constrain(speed1, MIN_ABSOLUTE_SPEED, 100);
  speed2 = constrain(speed2, MIN_ABSOLUTE_SPEED, 100);
    
  analogWrite(enableMotor1, speed1);
  analogWrite(enableMotor2, speed2);    
}
