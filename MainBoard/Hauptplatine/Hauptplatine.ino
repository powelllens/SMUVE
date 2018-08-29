/*
 Example sketch for the PS3 Bluetooth library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
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
//    PS3.getButtonClick(CROSS
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

//Stuff needed for PS3 controller
byte LeftY,RightY,LeftX;

void SetupLED(){
  PS3.setLedOff();
  switch (MainBoardMode){
    case 3:
      PS3.setLedOn(LED1);
      break;
    case 4:
      PS3.setLedOn(LED2);
      break;
    case 5:
      PS3.setLedOn(LED3);
      break;
  }
}

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

void GetSensorDistances(){
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
  MinFrontDistance = receivedCommands[0];
  for (int c = 1; c < 6; c++){
    if (receivedCommands[c] < MinFrontDistance){
      MinFrontDistance = receivedCommands[c];
    }
  } 
  MinBackDistance = receivedCommands[5];
  for (int c = 6; c < 10; c++){
    if (receivedCommands[c] < MinBackDistance){
      MinBackDistance = receivedCommands[c];
    }
  }
//  Serial.print(MinFrontDistance);
//  Serial.print(" - ");
//  Serial.println(MinBackDistance);
}

//**********************************************************************************************
// Stuff needed for Motor Modul
#define MOTORDRIVE_ADDRESS  0x08
byte modeRegister = 0;

double Motor1Moved;
bool Motot1Enabled;
bool Motor1Direction;
byte Motor1Speed;
double Motor1SpeedDouble;

double Motor2Moved;
bool Motor2Enabled;
bool Motor2Direction;
byte Motor2Speed;
double Motor2SpeedDouble;

double DistanzMotor1;
double DistanzMotor2;
double MotorDistanz1Error;
double MotorDistanz2Error;
byte MaxMotorSpeed;
const byte MotorMaxCurrent = 0x64;
bool MotorBusy;
bool RecievedCommand;

void GetMovedDistance(){
  //Send Byte 0x00 to prepare for recieve
  delayMicroseconds(20);
  Wire.beginTransmission(MOTORDRIVE_ADDRESS); // transmit to device #8
  Wire.write(0x00);
  Wire.endTransmission();    // stop transmitting
  
  //Request 4 Bytes
  delayMicroseconds(20);
  Wire.requestFrom(MOTORDRIVE_ADDRESS, 4);    // request 4 bytes from slave device #0x08
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
}

void InitMotorModule(){
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

void ResetMotorMovement(){
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

void serialEvent(){
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    MsgString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void getValue(String data, char separator){
  byte CompIndex = 0;
  int strIndex[] = {0, 1};
  int maxIndex = data.length()-1;
  int i;
  for(i=1; i<=maxIndex; i++){
    if(data.charAt(i)==separator){
        strIndex[1]=i;
        MsgComp[CompIndex] = data.substring(strIndex[0], strIndex[1]);
        strIndex[0]=strIndex[1] + 1;
        CompIndex++;
    }
  }
  strIndex[1]=i;
  MsgComp[CompIndex] = data.substring(strIndex[0], strIndex[1]);
}

//**********************************************************************************************
void setup(){
  //Watchdog 1s

  delay(500);
  wdt_enable(WDTO_2S);
  
  //Start Serial
  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  MsgString.reserve(50);
  
  //Start I2C
  Wire.begin(); // join i2c bus (address optional for master)
  Wire.setClock(400000);
  
  //Init MotorModule
  InitMotorModule();
  
  //Start PS3 Bluetooth
  if (Usb.Init() == -1) {
    Serial.println("OSC did not start");
  }
  Serial.println("Start");
}

//**********************************************************************************************
void loop(){
  Usb.Task();
  wdt_reset();
  
  if (PS3.PS3Connected) {    
    if (MainBoardMode == 0){
      MainBoardMode = 3;
      Motor1Direction = 0;
      Motor2Direction = 0;
      PS3.setLedOff();
      PS3.setLedOn(LED1);
    }
    
    if (PS3.getButtonClick(TRIANGLE)){
      MainBoardMode++;
      if (MainBoardMode > 5){
        MainBoardMode = 3;
      }
      SetupLED();
    }
    
    if (PS3.getButtonClick(CIRCLE)){
      MainBoardMode--;
      if (MainBoardMode < 3){
        MainBoardMode = 5;
      }
      SetupLED();
    }
      
    if (PS3.getButtonClick(PS)){
      Serial.println("PS");
      bitWrite(modeRegister, 0, 0);
      bitWrite(modeRegister, 1, 0);
      PS3.disconnect();
    }

    if (MainBoardMode == 3){
      if (PS3.getAnalogHat(LeftHatY) > 137 || PS3.getAnalogHat(LeftHatY) < 117){
        LeftY = PS3.getAnalogHat(LeftHatY); 
        if (LeftY > 137) {
          bitWrite(modeRegister, 2, 1);
          LeftY = LeftY - 137;
        }
        else{
          bitWrite(modeRegister, 2, 0);
          LeftY = 117 - LeftY;
        }
        bitWrite(modeRegister, 0, 1);
        Motor1Speed = LeftY;
        if (Motor1Speed < 5){
           Motor1Speed = 5;
        }
      }
      else{
        bitWrite(modeRegister, 0, 0);
      }
      
      if (PS3.getAnalogHat(RightHatY) > 137 || PS3.getAnalogHat(RightHatY) < 117){
        RightY = PS3.getAnalogHat(RightHatY);
        if (RightY > 137) {
          bitWrite(modeRegister, 3, 1);
          RightY = RightY - 137;
        }
        else{
          bitWrite(modeRegister, 3, 0);
          RightY = 117 - RightY;
        }
        bitWrite(modeRegister, 1, 1);
        Motor2Speed = RightY;
        if (Motor2Speed < 5){
          Motor2Speed = 5;
        }
      }
      else{
        bitWrite(modeRegister, 1, 0);
      } 
    }
    else if (MainBoardMode == 4){
      
      LeftY = PS3.getAnalogHat(LeftHatY);
      LeftX = PS3.getAnalogHat(LeftHatX);
      
      double leftmtr =  LeftX - LeftY;
      double rightmtr = LeftX + LeftY - 255;
      double biggest = max(abs(leftmtr), abs(rightmtr));
      
      if (biggest > 100){
        leftmtr = (leftmtr * 100.0) / biggest;
        rightmtr = (rightmtr * 100.0) / biggest;
      }

      if (leftmtr < 0){
        bitWrite(modeRegister, 2, 0);
      }
      else{
        bitWrite(modeRegister, 2, 1);
      }
      Motor1Speed = abs(leftmtr);
      if (Motor1Speed > 8){
        bitWrite(modeRegister, 0, 1);
      }
      else{
        bitWrite(modeRegister, 0, 0);
      }

      if (rightmtr < 0){
        bitWrite(modeRegister, 3, 1);
      }
      else{
        bitWrite(modeRegister, 3, 0);
      }
      Motor2Speed = abs(rightmtr);
      if (Motor2Speed > 8){
        bitWrite(modeRegister, 1, 1);
      }
      else{
        bitWrite(modeRegister, 1, 0);
      }
    }
    else if (MainBoardMode == 5){
      if (PS3.getButtonClick(L1)){
        Motor1Direction = not Motor1Direction;
        bitWrite(modeRegister, 2, Motor1Direction);
      }
      if (PS3.getButtonClick(R1)){
        Motor2Direction = not Motor2Direction;
        bitWrite(modeRegister, 3, Motor2Direction);
      }
      
      Motor1Speed = PS3.getAnalogButton(L2);
      if (Motor1Speed > 5){
        Motor1Speed = Motor1Speed / 2;
        bitWrite(modeRegister, 0, 1);
      }
      else{
        Motor1Speed = 0;
        bitWrite(modeRegister, 0, 0);
      }

      Motor2Speed = PS3.getAnalogButton(R2);
      if (Motor2Speed > 5){
        Motor2Speed = Motor2Speed / 2;
        bitWrite(modeRegister, 1, 1);
      }
      else{
        Motor2Speed = 0;
        bitWrite(modeRegister, 1, 0);
      }
    }
  }
  else{
    MainBoardMode = 0;
  }
  
  if (stringComplete){
    stringComplete = false;    
    // read the incoming byte:
    Serial.println(MsgString);
    if (MsgString.length() > 11){
      getValue(MsgString,';');
    }
    else if (MsgString.length() == 2){
      MsgComp[0] = MsgString;
    }
    else{
      MsgComp[0]="";
    }
    MsgString = "";

    switch(MsgComp[0].charAt(0)){
      case 0x42: // B
        if (MainBoardMode == 0){
          DistanzMotor1 = MsgComp[1].toInt();
          Motor1Direction = MsgComp[2].toInt();
  
          DistanzMotor2 = MsgComp[3].toInt();
          Motor2Direction = MsgComp[4].toInt();
  
          MaxMotorSpeed = MsgComp[5].toInt();

          RecievedCommand = true;
        }
        break;
      case 0x53: // S
        //Modus
        Serial.print(MainBoardMode);
        Serial.print(";");

        //Busy
        Serial.print(MotorBusy);
        Serial.print(";");

        //Block Motor 1
        Serial.print("0");
        Serial.print(";");
        Serial.print("0");
        Serial.print(";");
        //Block Motor 2
        Serial.print("0");
        Serial.print(";");
        Serial.println("0");
        break;
        return;
      case 0x52: // R
        //Kamikaze Reset of Arduino
        Serial.println("Reset");
        //wdt_enable(WDTO_15MS);
        break;
      default:
        return;
    }
  }

  if (RecievedCommand){
    RecievedCommand = false;
    ResetMotorMovement();
    MotorBusy = true;
    
    if(DistanzMotor1 > 10){
      bitWrite(modeRegister, 0, 1);
      bitWrite(modeRegister, 2, Motor1Direction);
    }
    else{
      bitWrite(modeRegister, 0, 0);
    }
    
    if(DistanzMotor2 > 10){
      bitWrite(modeRegister, 1, 1);
      bitWrite(modeRegister, 3, Motor2Direction);
    }
    else{
      bitWrite(modeRegister, 1, 0);
    }

    Motor1DistanzPID.SetOutputLimits(6, MaxMotorSpeed);
    Motor2DistanzPID.SetOutputLimits(6, MaxMotorSpeed);
    
    Motor1DistanzPID.SetMode(AUTOMATIC);
    Motor2DistanzPID.SetMode(AUTOMATIC);
  }
  
  GetMovedDistance();
  GetSensorDistances();

  if(MotorBusy){
    MotorDistanz1Error = DistanzMotor1 - Motor1Moved;
    MotorDistanz2Error = DistanzMotor2 - Motor2Moved;

    if(MotorDistanz1Error < 1){
      Motor1DistanzPID.SetMode(MANUAL);
      Motor1SpeedDouble = 0;
      bitWrite(modeRegister, 0, 0);
    }
    if(MotorDistanz2Error < 1){
      Motor2DistanzPID.SetMode(MANUAL);
      Motor2SpeedDouble = 0;
      bitWrite(modeRegister, 1, 0);
    }

    if ((MotorDistanz1Error < 1) && (MotorDistanz2Error < 1)){
      MotorBusy = false;
    }
    
//    Serial.print(Motor1Moved);
//    Serial.print(" - ");
//    Serial.print(Motor1SpeedDouble);
//    Serial.print(" - "); 
//    Serial.print(DistanzMotor1);
//    Serial.print(" - ");
//    Serial.print(Motor2Moved);
//    Serial.print(" - ");
//    Serial.print(Motor2SpeedDouble);
//    Serial.print(" - ");
//    Serial.println(DistanzMotor2);
//    
    Motor1DistanzPID.Compute();
    Motor1Speed = byte(Motor1SpeedDouble);
    Motor2DistanzPID.Compute();
    Motor2Speed = byte(Motor2SpeedDouble);
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
