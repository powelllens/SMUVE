/*

*/

#include <avr/wdt.h>
byte MainBoardMode = 0;

//**********************************************************************************************
//PS 3 Controller functionallity
#include <SPI.h>
#include <PS3BT.h>
#include <usbhub.h>

//    PS3.getButtonClick(L1)
//    PS3.getButtonClick(R1)
//    PS3.getAnalogButton(L2)
//    PS3.getAnalogButton(R2)
//
//    PS3.getButtonClick(TRIANGLE)
//    PS3.getButtonClick(CIRCLE)
//    PS3.getButtonClick(CROSS)
//    PS3.getButtonClick(SQUARE)
//
//    PS3.setRumbleOn(RumbleLow)
//    PS3.setRumbleOn(RumbleHigh)
//
//    PS3.getButtonClick(UP)
//    PS3.getButtonClick(RIGHT)
//    PS3.getButtonClick(DOWN)
//    PS3.getButtonClick(LEFT)
//
//    PS3.setLedOff()
//    PS3.setLedOn(LED1)
//    PS3.setLedOn(LED2)
//    PS3.setLedOn(LED3)
//    PS3.setLedOn(LED4)
//
//    PS3.getButtonClick(SELECT)
//    PS3.getButtonClick(START)
//
//    PS3.printStatusString()

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
PS3BT PS3(&Btd); // This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x1A, 0x7D, 0xDA, 0x71, 0x13); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch
//00:1A:7D:DA:71:13

//Variable Decleration:
//**********************************************************************************************
// I2C functionallity
#include <Wire.h>

// general I2C setup
#define MAX_RECIEVE_BYTES  10
byte receivedCommands[MAX_RECIEVE_BYTES];

//**********************************************************************************************
// Stuff needed for Ultrasonic Modul
#define ULTRASONIC_ADDRESS  0x09
byte MinFrontDistance;
byte MinBackDistance;

bool DistanceFrontBlock;
bool FrontRightBlock;
bool FrontLeftBlock;

bool BackRightBlock;
bool BackLeftBlock;

const int MinMotorSpeed = 12;
const byte SmallestDistanceToMove = 10; //Pulse
const byte SmallestDistanceToObjectFront = 30; //cm
const byte SmallestDistanceToObjectforAngledSensors = 100; //cm
const byte MaxDistanceToObject = 160; //cm
const int DeltaDistanceToObject = MaxDistanceToObject - SmallestDistanceToObjectFront; //cm
double Distancebuffer;
double DistanceSpeedFactor;

//**********************************************************************************************
// Stuff needed for Motor Modul
#define MOTORDRIVE_ADDRESS  0x08
byte modeRegister = 0;

double Motor1Moved;
bool Motot1Enabled;
bool Motor1Active;
bool Motor1Direction;
byte Motor1Speed;
double Motor1SpeedDouble;

double Motor2Moved;
bool Motor2Enabled;
bool Motor2Active;
bool Motor2Direction;
byte Motor2Speed;
double Motor2SpeedDouble;

double DistanzMotor1;
double DistanzMotor2;
double MotorDistanz1Error;
double MotorDistanz2Error;
const byte MotorMaxCurrent = 0x64;
bool MotorBusy;
bool RecievedCommand;

int MaxMotorSpeedDelta;
int MaxMotorSpeed;
int CalculatedMaxMotorSpeed;

//**********************************************************************************************
#include "PID_v1.h"

double Kp_D = 4;
double Ki_D = 4;
double Kd_D = 0;

PID Motor1DistanzPID(&Motor1Moved, &Motor1SpeedDouble, &DistanzMotor1, Kp_D, Ki_D, Kd_D, P_ON_M, DIRECT);
PID Motor2DistanzPID(&Motor2Moved, &Motor2SpeedDouble, &DistanzMotor2, Kp_D, Ki_D, Kd_D, P_ON_M, DIRECT);

//**********************************************************************************************
// Serial Communication
String MsgString = "";
String MsgComp[8] = String(5);
bool stringComplete = false;

//**********************************************************************************************
// Watchdog Hardware Reset
bool ResetSystem = false;


//Stuff needed for PS3 controller
byte LeftY, RightY, LeftX;
byte Speeddivider;
const byte ResetMAX3421Pin = 8;
bool PS3MotorFail;

void SetupLED(byte Mode) {
  PS3.setLedOff();
  switch (Mode) {
    case 2:
      PS3.setLedOn(LED1);
      break;
    case 3:
      PS3.setLedOn(LED2);
      break;
    case 4:
      PS3.setLedOn(LED3);
      break;
    case 5:
      PS3.setLedOn(LED4);
      break;
  }
}

void PS3Mode2getActions() {
  LeftY = PS3.getAnalogHat(LeftHatY);
  if (LeftY > 128) {
    Motor2Direction = 1;
    LeftY = LeftY - 128;
  }
  else {
    Motor2Direction = 0;
    LeftY = 128 - LeftY;
  }
  Motor2Speed = LeftY / Speeddivider;

  RightY = PS3.getAnalogHat(RightHatY);
  if (RightY > 128) {
    Motor1Direction = 0;
    RightY = RightY - 128;
  }
  else {
    Motor1Direction = 1;
    RightY = 128 - RightY;
  }
  Motor1Speed = RightY / Speeddivider;
}

void PS3Mode3getActions() {
  LeftY = PS3.getAnalogHat(LeftHatY);
  LeftX = PS3.getAnalogHat(LeftHatX);

  double rightmtr =  LeftX - LeftY;
  double leftmtr = LeftX + LeftY - 255;
  double biggest = max(abs(leftmtr), abs(rightmtr));

  if (biggest > 100) {
    leftmtr = (leftmtr * 100.0) / biggest;
    rightmtr = (rightmtr * 100.0) / biggest;
  }

  if (leftmtr < 0) {
    Motor1Direction = 1;
  }
  else {
    Motor1Direction = 0;
  }
  Motor1Speed = abs(leftmtr) / Speeddivider;

  if (rightmtr < 0) {
    Motor2Direction = 1;
  }
  else {
    Motor2Direction = 0;
  }

  Motor2Speed = abs(rightmtr) / Speeddivider;
}

void PS3Mode4getActions() {
  if (PS3.getButtonClick(R1)) {
    Motor1Direction = not Motor1Direction;
  }
  if (PS3.getButtonClick(L1)) {
    Motor2Direction = not Motor2Direction;
  }

  Motor1Speed = int(PS3.getAnalogButton(R2)) / Speeddivider;
  Motor2Speed = int(PS3.getAnalogButton(L2)) / Speeddivider;
}

//**********************************************************************************************
//Ultrasonic Modul

void GetSensorDistances() {
  //Send Byte 0x00 to prepare for recieve
  delayMicroseconds(20);
  Wire.beginTransmission(ULTRASONIC_ADDRESS); // transmit to device #8
  Wire.write(0x00);
  Wire.endTransmission();    // stop transmitting

  //Request 10 Bytes
  delayMicroseconds(20);
  Wire.requestFrom(ULTRASONIC_ADDRESS, 10);    // request 4 bytes from slave device #0x08
  byte a = 0;
  while (Wire.available()) { // slave may send less than requested
    receivedCommands[a] = Wire.read(); // receive a byte as character
    a++;
  }

  //First 0-3 are front sensors
  MinFrontDistance = receivedCommands[0];
  for (int c = 1; c < 4; c++) {
    if (receivedCommands[c] < MinFrontDistance) {
      MinFrontDistance = receivedCommands[c];
    }
  }

  //4-5 Light
  if (min(receivedCommands[4], receivedCommands[5]) < SmallestDistanceToObjectforAngledSensors) {
    DistanceFrontBlock = true;
  }
  else {
    DistanceFrontBlock = false;
  }

  // All Sesnors in the Back 6-9
  MinBackDistance = receivedCommands[6];
  for (int c = 7; c < 10; c++) {
    if (receivedCommands[c] < MinBackDistance) {
      MinBackDistance = receivedCommands[c];
    }
  }

  /*

       / | |   | | \
       4 0 1 | 2 3 5
       *************
     /7*           *6\
       *           *
       *           *
       *           *
       *************
       8           9
       \           /


  */

  if (MinBackDistance < SmallestDistanceToObjectforAngledSensors) {
    BackLeftBlock = true;
  }
  else {
    BackLeftBlock = false;
  }
  BackRightBlock = BackLeftBlock;

  if ((MinFrontDistance < SmallestDistanceToObjectFront) || DistanceFrontBlock) {
    FrontLeftBlock = true;
  }
  else {
    FrontLeftBlock = false;
  }
  FrontRightBlock = FrontLeftBlock;
}

//**********************************************************************************************
// Motor Modul

void GetMovedDistanceAndMode() {
  //Send Byte 0x00 to prepare for recieve
  delayMicroseconds(20);
  Wire.beginTransmission(MOTORDRIVE_ADDRESS); // transmit to device #8
  Wire.write(0x00);
  Wire.endTransmission();    // stop transmitting

  //Request 4 Bytes
  delayMicroseconds(20);
  Wire.requestFrom(MOTORDRIVE_ADDRESS, 5);    // request 5 bytes from slave device #0x08
  byte a = 0;
  while (Wire.available()) { // slave may send less than requested
    receivedCommands[a] = Wire.read(); // receive a byte as character
    a++;
  }
  word _Motor1moved = receivedCommands[0] << 8;
  _Motor1moved = _Motor1moved + receivedCommands[1];
  Motor1Moved = double(_Motor1moved);

  word _Motor2moved = receivedCommands[2] << 8;
  _Motor2moved = _Motor2moved + receivedCommands[3];
  Motor2Moved = double(_Motor2moved);

  if (bitRead(receivedCommands[4], 6) == 1) {
    MainBoardMode = 1;
  }
  else {
    if (MainBoardMode == 1) {
      MainBoardMode = 0;
    }
  }
  Motor1Active = bitRead(receivedCommands[4], 0);
  Motor2Active = bitRead(receivedCommands[4], 1);
}

void InitMotorModule() {
  //Motor Enable OFF
  bitWrite(modeRegister, 0, 0);
  bitWrite(modeRegister, 1, 0);
  //Motor Direction
  bitWrite(modeRegister, 2, Motor1Direction);
  bitWrite(modeRegister, 3, Motor2Direction);
  //Reset Motor movements
  bitWrite(modeRegister, 4, 1);
  bitWrite(modeRegister, 5, 1);

  delayMicroseconds(20);
  Wire.beginTransmission(MOTORDRIVE_ADDRESS); // transmit to device #8
  // StartAdress
  Wire.write(0x04);
  Wire.write(modeRegister);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(MotorMaxCurrent); //min 0x64 (dec 100 --> 3,5A)
  Wire.endTransmission();    // stop transmitting

  bitWrite(modeRegister, 4, 0);
  bitWrite(modeRegister, 5, 0);
}

void ResetMotorMovement() {
  bitWrite(modeRegister, 4, 1);
  bitWrite(modeRegister, 5, 1);

  delayMicroseconds(20);
  Wire.beginTransmission(MOTORDRIVE_ADDRESS); // transmit to device #8
  // StartAdress
  Wire.write(0x04);
  Wire.write(modeRegister);
  Wire.endTransmission();    // stop transmitting

  bitWrite(modeRegister, 4, 0);
  bitWrite(modeRegister, 5, 0);
}

void ResetMovementDistanze() {
  //Serial.println("final");
  if (not RecievedCommand) {
    DistanzMotor1 = 0;
    DistanzMotor1 = 0;
  }
  modeRegister = 0;
  ResetMotorMovement();
  GetMovedDistanceAndMode();
  Motor1DistanzPID.SetMode(MANUAL);
  Motor2DistanzPID.SetMode(MANUAL);
  Motor1SpeedDouble = 0;
  Motor2SpeedDouble = 0;
  MotorBusy = false;
}

//**********************************************************************************************
// Serial Communication

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();

    MsgString += inChar; // add it to the inputString:
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void getValue(String data, char separator) {
  byte CompIndex = 0;
  int strIndex[] = {0, 1};
  int maxIndex = data.length() - 1;
  int i;
  for (i = 1; i <= maxIndex; i++) {
    if (data.charAt(i) == separator) {
      strIndex[1] = i;
      MsgComp[CompIndex] = data.substring(strIndex[0], strIndex[1]);
      strIndex[0] = strIndex[1] + 1;
      CompIndex++;
    }
  }
  strIndex[1] = i;
  MsgComp[CompIndex] = data.substring(strIndex[0], strIndex[1]);
}


void ResetMax3421() {
  digitalWrite(ResetMAX3421Pin, LOW);
  delay(10);
  digitalWrite(ResetMAX3421Pin, HIGH);
  delay(10);
  if (Usb.Init() == -1) {
    Serial.println("OSC did not start");
  }
}
//**********************************************************************************************
void setup() {
  //Watchdog 1s
  pinMode(ResetMAX3421Pin, OUTPUT);

  //Enable Watchdog and set it to 8s
  //Watchdog will be reseted by Command "S" and PS3.Connected() if Reset of uC is switched off
  wdt_enable(WDTO_8S);

  //Start Serial
  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  MsgString.reserve(30);

  //Start I2C
  Wire.begin(); // join i2c bus (address optional for master)
  Wire.setClock(400000);

  //Init MotorModule
  InitMotorModule();

  //Start PS3 Bluetooth
  ResetMax3421();
  Serial.println("Start");

  MainBoardMode = 0;
}

//**********************************************************************************************
void loop() {
  Usb.Task();

  //  if (not ResetSystem){
  //    wdt_reset();
  //  }

  if (PS3.PS3Connected) {
    if (not ResetSystem) {
      wdt_reset();
    }
    if (MainBoardMode == 0) {
      MainBoardMode = 2;
      Motor1Direction = 0;
      Motor2Direction = 0;
      Speeddivider = 3;
      SetupLED(MainBoardMode);
    }
    else if (MainBoardMode == 1) {
      if (not PS3MotorFail) {
        PS3MotorFail = true;
        Motor1Direction = 0;
        Motor2Direction = 0;
        Motor2Speed = 0;
        Motor1Speed = 0;
        modeRegister = 0;
        SetupLED(5);
      }
    }

    if ((MainBoardMode > 1) && PS3MotorFail) {
      PS3MotorFail = false;
    }

    if (PS3.getButtonClick(PS)) {
      //Serial.println("PS");
      modeRegister = 0;
      Motor1Speed = 0;
      Motor2Speed = 0;
      PS3MotorFail = false;
      PS3.disconnect();
    }

    if (not PS3MotorFail) {
      if (PS3.getButtonClick(TRIANGLE)) {
        if (MainBoardMode > 3) {
          MainBoardMode = 2;
        }
        else {
          MainBoardMode++;
          if (MainBoardMode == 4) {
            Motor1Direction = 1;
            Motor2Direction = 0;
          }
        }
        SetupLED(MainBoardMode);
      }

      if (PS3.getButtonClick(CIRCLE)) {
        if (MainBoardMode < 3) {
          MainBoardMode = 4;
          Motor1Direction = 1;
          Motor2Direction = 0;
        }
        else {
          MainBoardMode--;
        }
        SetupLED(MainBoardMode);
      }

      if (PS3.getButtonClick(SELECT)) {
        if (Speeddivider < 2) {
          Speeddivider = 3;
        }
        else {
          Speeddivider--;
        }
      }
    }

    if ((MainBoardMode == 2) && (MainBoardMode != 1)) {
      PS3Mode2getActions();
    }

    else if ((MainBoardMode == 3) && (MainBoardMode != 1)) {
      PS3Mode3getActions();
    }

    else if ((MainBoardMode == 4) && (MainBoardMode != 1)) {
      PS3Mode4getActions();
    }

    if (Motor1Speed > 8) {
      bitWrite(modeRegister, 0, 1);
      bitWrite(modeRegister, 2, Motor1Direction);
    }
    else {
      Motor1Speed = 0;
      bitWrite(modeRegister, 0, 0);
    }

    if (Motor2Speed > 8) {
      bitWrite(modeRegister, 1, 1);
      bitWrite(modeRegister, 3, Motor2Direction);
    }
    else {
      Motor2Speed = 0;
      bitWrite(modeRegister, 1, 0);
    }

  }
  else {
    if (MainBoardMode > 1) {
      MainBoardMode = 0;
      ResetMotorMovement();
      bitWrite(modeRegister, 0, 0);
      bitWrite(modeRegister, 1, 0);
      Motor1Speed = 0;
      Motor2Speed = 0;
    }
    else if (MainBoardMode == 1) {
      Motor1Direction = 0;
      Motor2Direction = 0;
      Motor2Speed = 0;
      Motor1Speed = 0;
      modeRegister = 0;
    }
  }

  if (stringComplete) {
    stringComplete = false;
    // read the incoming byte:
    //Serial.print(MsgString);
    if (MsgString.length() > 11) {
      getValue(MsgString, ';');
    }
    else if (MsgString.length() == 2) {
      MsgComp[0] = MsgString;
    }
    else {
      MsgComp[0] = "";
    }
    MsgString = "";

    switch (MsgComp[0].charAt(0)) {
      case 0x42: // B
        wdt_reset();
        if ((MainBoardMode == 0) && (not MotorBusy)) {
          //Serial.print("Start");
          DistanzMotor1 = MsgComp[1].toInt();
          Motor1Direction = MsgComp[2].toInt();
          Motor1Direction = not Motor1Direction;

          DistanzMotor2 = MsgComp[3].toInt();
          Motor2Direction = MsgComp[4].toInt();

          MaxMotorSpeed = MsgComp[5].toInt();
          //Debug Slow Down
          if (MaxMotorSpeed > 60) {
            MaxMotorSpeed = 60;
          }
          else if (MaxMotorSpeed < MinMotorSpeed) {
            MaxMotorSpeed = MinMotorSpeed;
          }

          RecievedCommand = true;
        }
        break;
      case 0x53: // S
        if (not ResetSystem) {
          wdt_reset();
        }
        //Modus
        Serial.print(MainBoardMode);
        Serial.print(";");

        //Busy
        Serial.print(MotorBusy);
        Serial.print(";");

        //Block Motor 1
        Serial.print(FrontLeftBlock);
        Serial.print(";");
        Serial.print(FrontRightBlock);
        Serial.print(";");
        //Block Motor 2
        Serial.print(BackLeftBlock);
        Serial.print(";");
        Serial.println(BackRightBlock);
        break;
        return;
      case 0x52: // R
        //Kamikaze Reset of Arduino
        Serial.println("Reset");
        //wdt_enable(WDTO_15MS);
        ResetSystem = true;
        ResetMax3421();
        break;
      default:
        return;
    }
  }

  GetMovedDistanceAndMode();
  GetSensorDistances();

  if (RecievedCommand) {

    ResetMotorMovement();

    RecievedCommand = false;
    MotorBusy = true;

    if (SmallestDistanceToMove < DistanzMotor1) {
      bitWrite(modeRegister, 0, 1);
      bitWrite(modeRegister, 2, Motor1Direction);
      Motor1DistanzPID.SetMode(AUTOMATIC);
    }
    else {
      DistanzMotor1 = 0;
      bitWrite(modeRegister, 0, 0);
    }

    if (SmallestDistanceToMove < DistanzMotor2) {
      bitWrite(modeRegister, 1, 1);
      bitWrite(modeRegister, 3, Motor2Direction);
      Motor2DistanzPID.SetMode(AUTOMATIC);
    }
    else {
      DistanzMotor2 = 0;
      bitWrite(modeRegister, 1, 0);
    }
    // 15 = 30 - 15
    MaxMotorSpeedDelta = MaxMotorSpeed - MinMotorSpeed;
    // 0,1153 = 15 / 130
    DistanceSpeedFactor = (double)MaxMotorSpeedDelta / DeltaDistanceToObject;
  }

  if (MotorBusy) {

    Distancebuffer = MinFrontDistance;

    if (DistanzMotor1 == DistanzMotor2) {
      //VorwÃ¤rts fahren
      Distancebuffer = (Distancebuffer - 30) * DistanceSpeedFactor;
      if (FrontRightBlock || DistanceFrontBlock) {
        ResetMovementDistanze();
      }
      else {
        CalculatedMaxMotorSpeed = Distancebuffer + MinMotorSpeed;
      }
    }
    else if ((DistanzMotor1 > DistanzMotor2)) {
      //Linkskurve
      Distancebuffer = min(Distancebuffer, MinBackDistance);
      Distancebuffer = (Distancebuffer - 30) * DistanceSpeedFactor;
      if (FrontRightBlock || BackRightBlock || DistanceFrontBlock) {
        ResetMovementDistanze();
      }
      else {
        CalculatedMaxMotorSpeed = Distancebuffer + MinMotorSpeed;
      }
    }
    else {
      //Rechtskurve
      Distancebuffer = min(Distancebuffer, MinBackDistance);
      Distancebuffer = (Distancebuffer - 30) * DistanceSpeedFactor;
      if (FrontRightBlock || BackRightBlock || DistanceFrontBlock) {
        ResetMovementDistanze();
      }
      else {
        CalculatedMaxMotorSpeed = Distancebuffer + MinMotorSpeed;
      }
    }

    //CalculatedMaxMotorSpeed = MaxMotorSpeed;

    Motor1DistanzPID.SetOutputLimits(MinMotorSpeed - 1, CalculatedMaxMotorSpeed);
    Motor2DistanzPID.SetOutputLimits(MinMotorSpeed - 1, CalculatedMaxMotorSpeed);

    MotorDistanz1Error = DistanzMotor1 - Motor1Moved;
    MotorDistanz2Error = DistanzMotor2 - Motor2Moved;

    if (MotorDistanz1Error < 1) {
      Motor1DistanzPID.SetMode(MANUAL);
      Motor1SpeedDouble = 0;
      bitWrite(modeRegister, 0, 0);
    }
    if (MotorDistanz2Error < 1) {
      Motor2DistanzPID.SetMode(MANUAL);
      Motor2SpeedDouble = 0;
      bitWrite(modeRegister, 1, 0);
    }

    if ((MotorDistanz1Error < 1) && (MotorDistanz2Error < 1)) {
      ResetMovementDistanze();
    }

    Motor1DistanzPID.Compute();
    Motor1Speed = byte(Motor1SpeedDouble);
    Motor2DistanzPID.Compute();
    Motor2Speed = byte(Motor2SpeedDouble);

    //B;100;0;100;0;20

  }

  delayMicroseconds(20);
  Wire.beginTransmission(MOTORDRIVE_ADDRESS); // transmit to device #8
  // StartAdress
  Wire.write(0x04);
  //ModeRegister
  Wire.write(modeRegister);
  //Motor1Speed
  Wire.write(Motor1Speed);
  //Motor2Speed
  Wire.write(Motor2Speed);
  //MotorMaxCurrent
  Wire.write(MotorMaxCurrent); //min 0x64 (dec 100 --> 3,5A)
  Wire.endTransmission();    // stop transmitting

}
