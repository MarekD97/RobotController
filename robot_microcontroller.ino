#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#define MAX_CURRENT 6

Adafruit_PWMServoDriver servo;
SoftwareSerial bluetoothSerial(10, 11); //RX, TX

bool aggressiveMode = false;

unsigned long currentTime;
unsigned long driveTime;
unsigned long motionTime;

unsigned int driveDirection;
bool driveCommand = false;
bool motionCommand = false;

float servoArmTo[6];
float servoArm[6];

void setup()
{
  Serial.begin(9600);
  bluetoothSerial.begin(9600);

  servo = Adafruit_PWMServoDriver();
  servo.begin();
  servo.setPWMFreq(60);

  Serial.println("Launching...");
  delay(2000);

  currentTime = driveTime = motionTime = millis();

  //0 - base rotation
  //1 - arm first angle
  //2 - arm second angle
  //3 - arm third angle
  //4 - gripper rotation
  //5 - gripper width
  motionCommand = true;
  for (int i = 0; i < 6; i++)
  {
    servoArmTo[i] = 90;
    servoArm[i] = 89;
    setServoAngle(i, servoArm[i]);
    delay(500);
  }
  Serial.println("Robot is ready.");
}

void loop()
{
  //Receive data from bluetoth serial
  receiveBluetoothData();

  //robot driving
  if (driveCommand)
  {
    driveTime = millis();
    driveTo(driveDirection);
    driveCommand = false;
  }
  if (millis() > driveTime + 100)
  {
    stopDrive();
  }

  //robot arm moving
  if (motionCommand)
  {
    bool motion = false;
    for (int i = 0; i < 6; i++)
    {
      if (servoArm[i] < servoArmTo[i] + 1 && servoArm[i] > servoArmTo[i] - 1)
        continue;
      if (servoArmTo[i] > servoArm[i])
      {
        motion = true;
        servoArm[i] += 1.5;
      }
      else if (servoArmTo[i] < servoArm[i])
      {
        motion = true;
        servoArm[i] -= 1.5;
      }
      setServoAngle(i, servoArm[i]);
    }
    if (motion == false)
      motionCommand = false;
  }

  //Read current value
  float currentValue = getCurrentValue();

  //Send current value to android device
  if (millis() > currentTime + 200)
  {
    bluetoothSerial.print("C");
    bluetoothSerial.print(currentValue, 3);
    bluetoothSerial.print(";");
    currentTime = millis();
  }

  // if (currentValue > 3)
  //   Serial.println(currentValue, 3);

  //Reset servos if current value will be too high
  if (currentValue > MAX_CURRENT)
  {
    Serial.println("DANGER");
    resetAllServos();
    delay(2000);
  }
}

void setServoAngle(int nr, float angle)
{
  Serial.print("Arm moving: ");
  Serial.print(nr);
  Serial.print(", angle: ");
  Serial.println(angle);
  if (nr == 0)
    angle -= 20;
  if (nr == 1)
    angle -= 5;
  if (nr == 4)
    angle -= 8;
  if (nr == 5)
    angle = angle * 3 / 5 + 20;
  int freq = int(2.5 * angle) + 150;
  servo.setPWM(nr, 0, freq);
}

void resetAllServos()
{
  Serial.println("Stopping all servos...");
  driveCommand = motionCommand = false;
  for (int i = 0; i < 12; i++)
  {
    servo.setPWM(i, 0, 0);
  }
  bluetoothSerial.print("STOP");
}

float getCurrentValue()
{
  float voltage = -((5.0 / 1023.0) * analogRead(A3) - 2.5);
  float current = voltage / 0.100;
  return current;
}

void receiveBluetoothData()
{
  String buffer = getBluetoothData();
  if (buffer != "")
    Serial.println(buffer);
  for (int i = 0; i < 2; i++)
  {
    if (buffer.startsWith("A"))
    {
      Serial.println(buffer.substring(0, buffer.indexOf(";") + 1));
      unsigned int posId = buffer.substring(1, buffer.indexOf(":")).toInt();
      unsigned int value = buffer.substring(buffer.indexOf(":") + 1, buffer.indexOf(";")).toInt();
      buffer = buffer.substring(buffer.indexOf(";" + 1));

      motionCommand = true;
      if (posId < 6)
      {
        if (aggressiveMode)
          setServoAngle(posId, value * 1.8);
        else
          servoArmTo[posId] = value * 1.8;
      }
    }
    else if (buffer.startsWith("D"))
    {
      driveDirection = buffer.substring(1, 2).toInt();
      buffer = buffer = buffer.substring(2);
      driveCommand = true;
      Serial.print("Drive to: ");
      Serial.println(driveDirection);
    }
    else if (buffer.startsWith("STOP"))
    {
      resetAllServos();
      buffer = buffer.substring(4);
    }
    else if (buffer.startsWith("GET_PARAMS"))
    {
      for (int i = 0; i < 6; i++)
      {
        delay(150);
        bluetoothSerial.print("A");
        bluetoothSerial.print(i);
        bluetoothSerial.print(":");
        bluetoothSerial.print(servoArmTo[i]);
        bluetoothSerial.print(";");
      }
      buffer = buffer.substring(10);
    }
    else if (buffer.startsWith("M"))
    {
      int mode = buffer.substring(1, 2).toInt();
      buffer = buffer.substring(2);
      if (mode == 1)
        aggressiveMode = true;
      else
        aggressiveMode = false;
    }
    else
      buffer = buffer.substring(1);
  }
}

String getBluetoothData()
{
  String buffer = "";
  while (bluetoothSerial.available())
  {
    char character = bluetoothSerial.read();
    buffer += character;
    if (buffer.length() > 10)
      break;
  }
  return buffer;
}

void driveTo(int direction)
{
  unsigned int left;
  unsigned int right;

  switch (direction)
  {
  case 0: //move forward
    left = 300;
    right = 400;
    break;
  case 1: //move backwards
    left = 400;
    right = 300;
    break;
  case 2: //rotate left
    left = right = 300;
    break;
  case 3: //rotate right
    left = right = 400;
    break;
  default:
    left = right = 0;
    break;
  }
  servo.setPWM(8, 0, left);
  servo.setPWM(9, 0, right);
  servo.setPWM(10, 0, left);
  servo.setPWM(11, 0, right);
  // Serial.println("Drive to");
  // Serial.println(left);
  // Serial.println(right);
}

void stopDrive()
{
  servo.setPWM(8, 0, 0);
  servo.setPWM(9, 0, 0);
  servo.setPWM(10, 0, 0);
  servo.setPWM(11, 0, 0);
  // Serial.println("Stop driving");
}