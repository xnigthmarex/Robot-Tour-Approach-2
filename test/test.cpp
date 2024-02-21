#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// MPU6050 variables
MPU6050 mpu;
bool dmpReady = false;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
int16_t gyro[3];
const int INTERRUPT_PIN = 2;
volatile bool mpuInterrupt = false;
float target_angle = 0;

// Motor Driver
const int enA = 11; // right
const int in1 = 10;
const int in2 = 9;
const int enB = 6; // left
const int in4 = 7;
const int in3 = 8;
int left_speed = 165;
int right_speed = 165;

// encoder
const int encoderleft = 15;
const int encoderright = 14;
volatile int leftcount = 0;
volatile int rightcount = 0;
volatile int prev_leftcount = 0;
volatile int prev_rightcount = 0;
int target_count = 0;

// encoder variables
const float encoder_resolution = 21.0;
const float wheel_diameter = 6.8;
const float wheel_circumference = 3.14159 * wheel_diameter;

// Ultrasonic sensor variables
const int TRIG = 13;
const int ECHO = 12;
long duration;
int sum_distance;
const int PRE_DEF_ERROR_VAL = 400;

// LED - PINS AND STATES
const int blue_led = 19;
const int red_led = 21;
const int green_led = 20;
boolean blue_led_state = false;
boolean red_led_state = false;
boolean green_led_state = false;

// LED - FUNCTIONS
void blue();
void red();
void green();
void reset();

// switch
const int switchPin = 16;
const int ALL_WAYS_ON = 17;
volatile bool switchPressed = false;

// function proto
void left();
void right();
void straight();
void stop();
void rotate(int target_angle, float angle, float angular_velocity);
//void straight(int hold_angle, int angle, int target_count);
void straight(float distance);
int distance(); // average of 5 without PRE_DEF_ERROR_VAL

// array of int commands
int commands[] = {100};
int command_length = sizeof(commands) / sizeof(commands[0]) - 1;
int command_index = 0;
bool program_stop = false;

void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
  // LED
  pinMode(blue_led, OUTPUT);
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  red();

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

  // Motor Driver
  pinMode(enA, OUTPUT); // left
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); // right
  pinMode(in4, OUTPUT);
  pinMode(in3, OUTPUT);

  // Switch
  pinMode(switchPin, INPUT_PULLDOWN);
  pinMode(ALL_WAYS_ON, OUTPUT);
  attachInterrupt(
      digitalPinToInterrupt(switchPin), []()
      { switchPressed = true; },
      RISING);
  digitalWrite(ALL_WAYS_ON, HIGH);
  switchPressed = false; // IMP

  // Ultrasonic sensor
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);


  reset();
  green();
}

void setup1() // core 2
{
  pinMode(encoderleft, INPUT_PULLUP);
  pinMode(encoderright, INPUT_PULLUP);
  attachInterrupt(
      digitalPinToInterrupt(encoderleft), []()
      { leftcount++; },
      RISING);
  attachInterrupt(
      digitalPinToInterrupt(encoderright), []()
      { rightcount++; },
      RISING);
}

void loop()
{
  if (switchPressed)
  {

    if (!dmpReady)
      return;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetGyro(gyro, fifoBuffer);
      float angle = ypr[0] * 180 / M_PI;
      float angular_velocity = gyro[2];

      if (commands[command_index] != 0) // or != 90
      {
        if (angle < 0)
        {
          angle = 360 + angle;
        }
      }
      if (command_index > command_length)
      {
        switchPressed = false;
        command_index = 0;
        return;
      }
      else
      {
        if ((commands[command_index]) % 90 == 0)
        {
          target_angle = commands[command_index];
          rotate(target_angle, angle, angular_velocity);
        }
        else
        {

          float numRev = (commands[command_index] / wheel_circumference);
          target_count = round(numRev * encoder_resolution) + 1;
          //straight(target_angle, angle, target_count);
          straight(commands[command_index]);
        }
      }
    }
  }
}

void rotate(int target_angle, float angle, float angular_velocity)
{
  int deltaAngle = round(target_angle - angle);
  int target_angular_velocity;

  if ((abs(deltaAngle) <= 1) && abs(angular_velocity) == 0) // IMPORTANT
  {
    stop();
    command_index++;
    delay(2000);
    return;
  }
  else
  {
    if (angle > target_angle)
    {
      left();
    }
    else
    {
      right();
    }
  }

  if (abs(deltaAngle) > 30)
  {
    target_angular_velocity = 48;
  }
  else
  {
    target_angular_velocity = 2 * abs(deltaAngle);
  }

  if (round(target_angular_velocity - abs(angular_velocity)) == 0)
  {
    ;
  }
  else if (target_angular_velocity - abs(angular_velocity) > 0)
  {
    left_speed++;
  }
  else
  {
    left_speed--;
  }
  left_speed = constrain(left_speed, 0, 220);
  right_speed = left_speed;
  analogWrite(enB, right_speed);
  analogWrite(enA, left_speed);
}

void straight(float distance)
{
  // use encoders counts
  straight();
  int leftPower = 190;
  int rightPower = 200;
  int offset = 5;

  if (distance < 0)
  {
    leftPower *= -1;
    rightPower *= -1;
    offset *= -1;
  }

  float correction = 1.0;
  if (distance > 0)
    distance += correction;
  else if (distance < 0)
    distance -= correction;

  leftcount = 0;
  rightcount = 0;
  prev_leftcount = 0;
  prev_rightcount = 0;
  int left_diff, right_diff;

  float countsPerRev = 21.0;
  float wheelDiameter = 6.8;
  float wheel_cir = wheelDiameter * 3.14159;

  float numRev = (distance / wheel_cir) + 1.0;
  // round up target count int
  int target_count = round(numRev * countsPerRev);

  delay(100);

  while (abs(rightcount) < abs(target_count))
  {
    leftPower = 195;  // most stupid line - but it needs to be there
    rightPower = 208; // most stupid line - but it needs to be there
    left_diff = abs(leftcount - prev_leftcount);
    right_diff = abs(rightcount - prev_rightcount);

    prev_leftcount = leftcount;
    prev_rightcount = rightcount;

    if (left_diff > right_diff)
    {
      leftPower = leftPower - (offset);
      rightPower = rightPower + (offset);
    }
    else if (right_diff > left_diff)
    {
      leftPower = leftPower + (offset);
      rightPower = rightPower - (offset);
    }

    Serial.println(left_diff);
    Serial.println(right_diff);
    Serial.println(leftPower);
    Serial.println(rightPower);
    Serial.println(" ");
    analogWrite(enA, leftPower);
    analogWrite(enB, rightPower);

    delay(10);
  }
  stop();
  command_index++;
  delay(500);
}

// void straight(int hold_angle, int angle, int target_count)
// {
//   straight();
//   Serial.println(target_count);
//   leftcount = 0;
//   rightcount = 0;

//   int leftPower = 195;
//   int rightPower = 203;
//   int offset = 7;

//   int left_diff, right_diff;

//   while (abs(rightcount) < abs(target_count))
//   {
//     leftPower = 195;
//     rightPower = 203;

//     left_diff = abs(leftcount - prev_leftcount);
//     right_diff = abs(rightcount - prev_rightcount);

//     prev_leftcount = leftcount;
//     prev_rightcount = rightcount;

//     if (left_diff > right_diff)
//     {
//       leftPower = leftPower - (offset);
//       rightPower = rightPower + (offset);
//     }
//     else if (right_diff > left_diff)
//     {
//       leftPower = leftPower + (offset);
//       rightPower = rightPower - (offset);
//     }

//     analogWrite(enA, leftPower);
//     analogWrite(enB, rightPower);

//     delay(5);
//   }
//   stop();
//   command_index++;
//   delay(2000);
// }

int distance()
{
  int count = 0;
  int sum = 0;
  while (count < 5)
  {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    duration = pulseIn(ECHO, HIGH);
    sum_distance = (duration * 0.034) / 2;
    if(sum_distance > 300){
      break;
    }
    sum = sum + sum_distance;
    count++;
  }
  return (sum / 5);
}

// motor functions
void right()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void left()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void straight()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
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
