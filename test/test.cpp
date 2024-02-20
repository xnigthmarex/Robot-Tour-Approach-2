#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// MPU6050 variables
MPU6050 mpu;
#define INTERRUPT_PIN 2
volatile bool mpuInterrupt = false;
bool dmpReady = false;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
int16_t gyro[3];


// Motor Driver
const int enA = 11;
const int in1 = 10;
const int in2 = 9;
const int enB = 6;
const int in4 = 7;
const int in3 = 8;

// functions proto
void left();
void right();
void m1f(int speed);
void m2f(int speed);
void m1b(int speed);
void m2b(int speed);
void stop();

// LED - PINS AND STATES
const int blue_led = 19;
const int red_led = 20;
const int green_led = 21;
boolean blue_led_state = false;
boolean red_led_state = false;
boolean green_led_state = false;

// LED - FUNCTIONS
void blue();
void red();
void green();
void reset();
bool shouldStop = false; 

// test
int left_speed = 190;
int right_speed = 190;
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setXAccelOffset(-3699);
  mpu.setYAccelOffset(-2519);
  mpu.setZAccelOffset(1391);
  mpu.setXGyroOffset(-156);
  mpu.setYGyroOffset(-11);
  mpu.setZGyroOffset(-14);
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.setDLPFMode(MPU6050_DLPF_BW_42); 
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  mpu.setDMPEnabled(true);
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  

  pinMode(blue_led, OUTPUT);
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
}

void loop()
{
  if (!dmpReady || shouldStop) // Check if the flag is set to true
    return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGyro(gyro, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    float angle = ypr[0] * 180 / M_PI;
    float angular_velocity = gyro[2];

    int deltaAngle = round(90 - angle);
    int target_angular_velocity;

    if((abs(deltaAngle)<=1) && abs(angular_velocity) == 0){
      stop();
      blue();
      shouldStop = true;
      return;
    }
    else{
      if(angle > 90){
        right();
      }
      else{
        left();
      }
    }

    if(abs(deltaAngle) > 30){
      target_angular_velocity = 60;
    }
    else{
      target_angular_velocity = 2 * abs(deltaAngle);
    }

    if(round(target_angular_velocity - abs(angular_velocity)) == 0){
      ;
    }
    else if(target_angular_velocity - abs(angular_velocity) > 0){
      left_speed++;
    }
    else{
      left_speed--;
    }
    constrain(left_speed, 180, 220);
    right_speed = left_speed;
    analogWrite(enB, right_speed);
    analogWrite(enA, left_speed);

  }
}

// motor functions
void left(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
void right(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}
void m1f(int speed)
{
  analogWrite(enA, abs(speed));
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void m2f(int speed)
{
  analogWrite(enB, abs(speed));
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void m1b(int speed)
{
  analogWrite(enA, abs(speed));
  digitalWrite(in2, LOW);
  digitalWrite(in1, HIGH);
}

void m2b(int speed)
{
  analogWrite(enB, abs(speed));
  digitalWrite(in4, LOW);
  digitalWrite(in3, HIGH);
}

void stop()
{
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}


void blue()
{
  digitalWrite(blue_led, !blue_led_state);
  blue_led_state = !blue_led_state;
}

void red()
{
  digitalWrite(red_led, !red_led_state);
  red_led_state = !red_led_state;
}

void green()
{
  digitalWrite(green_led, !green_led_state);
  green_led_state = !green_led_state;
}

void reset()
{
  digitalWrite(blue_led, LOW);
  digitalWrite(red_led, LOW);
  digitalWrite(green_led, LOW);
  blue_led_state = false;
  red_led_state = false;
  green_led_state = false;
}
