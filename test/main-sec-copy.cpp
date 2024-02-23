#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <atomic>


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
volatile uint32_t leftcount = 0;
volatile uint32_t rightcount = 0;
int target_count = 0;

// encoder variables
const float encoder_resolution = 20.0;
const float wheel_diameter = 6.8;
const float wheel_circumference = 3.14159 * wheel_diameter;

// Ultrasonic sensor variables
const int TRIG = 13;
const int ECHO = 12;
long duration;
int sum_distance;
const int PRE_DEF_ERROR_VAL = 400;
const int PRE_DEV_TARGET_DISTANCE = 15; // TODO cm value for predefined distance

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
void travel(int hold_angle, float angle, float angular_velocity, int target_count, int right_count);
// overloaded function
void travel(int hold_angle, float angle, float angular_velocity, int current_distance);
int distance(); // average of 5 without PRE_DEF_ERROR_VAL

// array of int commands
int commands[] = {1000,90,1000,180,1000,270,1000,0,1000};
int command_length = sizeof(commands) / sizeof(commands[0]) - 1;
int command_index = 0;
bool program_stop = false;

// temporary delay
bool delay_temp = false;

void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
  // LED
  pinMode(blue_led, OUTPUT_2MA);
  pinMode(red_led, OUTPUT_2MA);
  pinMode(green_led, OUTPUT_2MA);
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
      {
        switchPressed = true;
        delay_temp = true; },
      RISING);
  digitalWrite(ALL_WAYS_ON, HIGH);
  switchPressed = false; // IMP

  // Ultrasonic sensor
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // converting  distance(cm) to encoder count
  for (int i = 0; i <= command_length; i++)
  {
    if ((!(commands[i] >= 1000)) && (!(commands[i] % 90 == 0))) // TODO point of failure - may be
    {

      float numRev = (commands[i] / wheel_circumference);
      commands[i] = round(numRev * encoder_resolution);
    }
  }
  reset();
  green();
  delay(1000);
  reset();
}

void setup1()
{
  // pinMode(encoderright, INPUT_PULLUP);
  // attachInterrupt(
  //     digitalPinToInterrupt(encoderright), []()
  //     {
  //       rightcount++;
  //       rp2040.fifo.push(rightcount);
  //     },
  //     RISING); //TODO check the interrupt type
  // pinMode(encoderleft, INPUT_PULLUP);
  // attachInterrupt(
  //     digitalPinToInterrupt(encoderleft), []()
  //     {
  //       leftcount++;
  //       rp2040.fifo.push(leftcount);
  //     },
  //     RISING);  //TODO check the interrupt type

  pinMode(encoderright, INPUT_PULLUP);
  attachInterrupt(
      digitalPinToInterrupt(encoderright), []()
      { rightcount++; },
      FALLING);
}

void loop()
{
  if (switchPressed)
  {
    if (delay_temp)
    {
      delay(2000);
      delay_temp = false;
    }

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
          if (commands[command_index] != 0) // TODO or != 90
          {
            if (angle < 0)
            {
              angle = 360 + angle;
            }
          }
          target_angle = commands[command_index];
          rotate(target_angle, angle, angular_velocity);
        }
        else
        {
          if (target_angle != 0)
          {
            if (angle < 0)
            {
              angle = 360 + angle;
            }
          }
          if (commands[command_index] >= 1000)
          {
            int current_distance = distance();
            travel(target_angle, angle, angular_velocity, current_distance);
          }
          else
          {
            target_count = commands[command_index];
            int right_count = rightcount;
            Serial.println(right_count);
            travel(target_angle, angle, angular_velocity, target_count, right_count);
          }
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
    left_speed = 210;
    right_speed = 200;
    rightcount = 0;
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
    target_angular_velocity = 50;
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

void travel(int hold_angle, float angle, float angular_velocity, int target_count, int right_count)
{
  straight();

  if (target_count <= right_count)
  {
    stop();
    command_index++;
    rightcount = 0;
    left_speed = 200;
    right_speed = 200;
    delay(2000);
    return;
  }
  else
  {
    int delta_angle = round(hold_angle - angle);
    int target_angular_velocity;

    if (delta_angle > 30)
    {
      target_angular_velocity = 60;
    }
    else if (delta_angle < -30)
    {
      target_angular_velocity = -60;
    }
    else
    {
      target_angular_velocity = 2 * delta_angle;
    }
    angular_velocity = -1 * angular_velocity; // gyro data and accelerometer data are opposite
    if (round(target_angular_velocity - angular_velocity) == 0)
    {
      ;
    }
    else if (target_angular_velocity > angular_velocity)
    {
      left_speed++;
    }
    else
    {
      left_speed--;
    }

    left_speed = constrain(left_speed, 0, 210);
    right_speed = 193;
    analogWrite(enA, left_speed);
    analogWrite(enB, right_speed);
  }
}

// overloaded function - to use ultrasonic sensor
void travel(int hold_angle, float angle, float angular_velocity, int current_distance)
{
  straight();
  Serial.println(current_distance);
  if (PRE_DEV_TARGET_DISTANCE >= current_distance)
  {
    stop();
    command_index++;
    rightcount = 0;
    left_speed = 190;
    right_speed = 200;
    delay(2000);
    return;
  }
  else
  {
    int delta_angle = round(hold_angle - angle);
    int target_angular_velocity;

    if (delta_angle > 30)
    {
      target_angular_velocity = 60;
    }
    else if (delta_angle < -30)
    {
      target_angular_velocity = -60;
    }
    else
    {
      target_angular_velocity = 2 * delta_angle;
    }
    angular_velocity = -1 * angular_velocity; // gyro data and accelerometer data are opposite
    if (round(target_angular_velocity - angular_velocity) == 0)
    {
      ;
    }
    else if (target_angular_velocity > angular_velocity)
    {
      left_speed++;
    }
    else
    {
      left_speed--;
    }

    left_speed = constrain(left_speed, 0, 210);
    right_speed = 193;
    analogWrite(enA, left_speed);
    analogWrite(enB, right_speed);
  }
}

int distance() // TODO validate the function
{
  // int count = 0;
  // int sum = 0;
  // while (count < 5)
  // {
  //   digitalWrite(TRIG, LOW);
  //   delayMicroseconds(2);
  //   digitalWrite(TRIG, HIGH);
  //   delayMicroseconds(10);
  //   digitalWrite(TRIG, LOW);

  //   duration = pulseIn(ECHO, HIGH);
  //   sum_distance = (duration * 0.034) / 2;
  //   if (sum_distance > 300)
  //   {
  //     break;
  //   }
  //   sum = sum + sum_distance;
  //   count++;
  // }
  // return (sum / 5);

  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  duration = pulseIn(ECHO, HIGH);
  sum_distance = (duration * 0.034) / 2;

  return sum_distance;
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