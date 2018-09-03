/*

*/

#include <MotorDriver.h>
#include <avr/wdt.h>
#include <Wire.h>

#define Debug 0

//Constant Variables
// Motorpins 1
const byte Motor1EnablePin = 4;
const byte Motor1PWMPin = 3;
const byte Motor1InputAPin = 6;
const byte Motor1InputBPin = 2;
const byte Motor1SensePin = A0;
const byte Motor1RotationPin = 9;

// Motorpins 2
const byte Motor2EnablePin = 5;
const byte Motor2PWMPin = 11;
const byte Motor2InputAPin = 7;
const byte Motor2InputBPin = 10;
const byte Motor2SensePin = A1;
const byte Motor2RotationPin = 8;

//Runtime Motor 1
bool Motor1Error;
bool Motor1Direction;
double Motor1Speed; //Motorfrequenzy 
bool Motor1Enabled;

//Runtime Motor 2
bool Motor2Error;
bool Motor2Direction;
double Motor2Speed; //Motorfrequenzy 
bool Motor2Enabled;

bool MotorERROR;

double MotorMaxCurrent = 100;

MotorDriver MotorRechts(55.9, 2.0, 12.0, 0.02, 2.0, 3.0, 0.0, &Motor1Speed, &MotorMaxCurrent, &Motor1Error, &Motor1Direction, &Motor1Enabled, Motor1EnablePin, Motor1PWMPin, Motor1InputAPin, Motor1InputBPin, Motor1SensePin, Motor1RotationPin);
MotorDriver MotorLinks(58.6, 2.0, 12.0, 0.02, 2.0, 3.0, 0.0, &Motor2Speed, &MotorMaxCurrent, &Motor2Error, &Motor2Direction, &Motor2Enabled, Motor2EnablePin, Motor2PWMPin, Motor2InputAPin, Motor2InputBPin, Motor2SensePin, Motor2RotationPin);

/* Wire variables */
#define SLAVE_ADDRESS   0x08
#define REG_MAP_SIZE    9
#define MAX_SENT_BYTES  5

/********* Global  Variables  ***********/
byte zeroA, zeroB, zeroC, zeroD;
byte zeroAData, zeroBData, zeroCData, zeroDData;

//Register byte
byte modeRegister = 0;
byte registerMap[REG_MAP_SIZE];
byte registerMapTemp[REG_MAP_SIZE - 1];
byte receivedCommands[MAX_SENT_BYTES];

void requestEvent(){
  wdt_reset();
  for (int c = 0; c < (REG_MAP_SIZE-1); c++){
    registerMap[c] = registerMapTemp[c];
  }
  //Set the buffer up to send all 14 bytes of data
  //Wire.write(registerMap+receivedCommands[0], REG_MAP_SIZE);
  Wire.write(registerMap+receivedCommands[0], REG_MAP_SIZE);
}

void receiveEvent(int bytesReceived){
  wdt_reset();
  for (int a = 0; a < bytesReceived; a++){
    if ( a < MAX_SENT_BYTES){
      receivedCommands[a] = Wire.read();
    }
    else{
      Wire.read();  // if we receive more data then allowed just throw it away
    }
  }
  if(bytesReceived == 1 && (receivedCommands[0] < REG_MAP_SIZE)){
    return; 
  }
  if(bytesReceived == 1 && (receivedCommands[0] >= REG_MAP_SIZE)){
    receivedCommands[0] = 0x00;
    return;
  }
  int CounterIndex = 0;
  switch(receivedCommands[0]){
    case 0x04:
      zeroA = 1; //status flag to let us know we have new data in register 0x0B
      zeroAData = receivedCommands[1]; //save the data to a separate variable
      bytesReceived--;
      if(bytesReceived == 1){  //only two bytes total were sent so we’re done
        return;
        break;
      }
      CounterIndex++;
    case 0x05:
      zeroB = 1;
      zeroBData = receivedCommands[1+CounterIndex];
      bytesReceived--;
      if(bytesReceived == 1){  //only two bytes total were sent so we’re done
        return;
        break;
      }
      CounterIndex++;
    case 0x06:
      zeroC = 1;
      zeroCData = receivedCommands[1+CounterIndex];
      bytesReceived--;
      if(bytesReceived == 1){  //only two bytes total were sent so we’re done
        return;
        break;
      }
      CounterIndex++;
    case 0x07:
      zeroD = 1;
      zeroDData = receivedCommands[1+CounterIndex];
      return; //we simply return here because 0x0C is the last writable register
      break;
    default:
      return; // ignore the commands and return
  }
}

void storeData(){
     byte * bytePointer;  //we declare a pointer as type byte
     byte arrayIndex = 0; //we need to keep track of where we are storing data in the array
     bytePointer = (byte*)&MotorRechts.MotorMoved; //latitude is 2 bytes

     for (int i = 1; i > -1; i--){
      registerMapTemp[arrayIndex] = bytePointer[i];  //increment pointer to store each byte
      arrayIndex++;
     }

     bytePointer = (byte*)&MotorLinks.MotorMoved; //longitude is 2 bytes
     
     for (int i = 1; i > -1; i--){
      registerMapTemp[arrayIndex] = bytePointer[i];  //increment pointer to store each byte
      arrayIndex++;
     }
     // Prepare Mode Register
     modeRegister = 0;
     bitWrite(modeRegister, 0, Motor1Enabled);
     bitWrite(modeRegister, 1, Motor2Enabled);
     bitWrite(modeRegister, 2, Motor1Direction);
     bitWrite(modeRegister, 3, Motor2Direction);
     
     bitWrite(modeRegister, 6, MotorERROR);

     registerMapTemp[arrayIndex] = modeRegister;
     arrayIndex++;
     registerMapTemp[arrayIndex] = byte(Motor1Speed);
     arrayIndex++;
     registerMapTemp[arrayIndex] = byte(Motor2Speed);
     arrayIndex++;
     registerMapTemp[arrayIndex] = byte(MotorMaxCurrent);
}

void changeModeConfig(){
  if(zeroA){
    zeroA = 0;
    Motor1Enabled = bitRead(zeroAData, 0);
    Motor2Enabled = bitRead(zeroAData, 1);
    Motor1Direction = bitRead(zeroAData, 2);
    Motor2Direction = bitRead(zeroAData, 3);
    
    if(bitRead(zeroAData, 4)){
      MotorRechts.MotorMoved = 0;
    }
    if(bitRead(zeroAData, 5)){
      MotorLinks.MotorMoved = 0;
    }
  }
  if(zeroB){
    Motor1Speed = double(zeroBData);
    zeroB = 0;
  }
  if(zeroC){
    Motor2Speed = double(zeroCData);
    zeroC = 0;
  }
  if(zeroD){
    MotorMaxCurrent = double(zeroDData);
    zeroD = 0;
  }
}

void setup(){
  #if Debug
  Serial.begin(115200);
  #endif
  Wire.begin(SLAVE_ADDRESS);    // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent);
  Wire.setClock(400000);
  
  //PWM Timer 2
  wdt_enable(WDTO_1S);
  //PWM Timer 2
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  OCR2A = 0;
  OCR2B = 0;

  modeRegister = 0;
  Motor1Speed = 0;
  Motor2Speed = 0;
  MotorMaxCurrent = 0;
  MotorERROR=false;
}

void loop(){
  
  if(Motor1Error || Motor2Error){
    MotorERROR = true;
    Motor1Enabled = false;
    Motor2Enabled = false;
    Motor1Speed = 0;
    Motor2Speed = 0;
  }
  
  if(zeroA || zeroB || zeroC || zeroD){
    changeModeConfig();  //call the function to make your changes
    #if Debug
    Serial.print(MotorRechts._SpeedIntegralMotor);
    Serial.print(" - ");
    Serial.print(int(MotorRechts._MotorSpeedInput));
    Serial.print(" - ");
    Serial.print(MotorRechts._MotorSpeedOutput);
    Serial.print(" - ");
    Serial.println(MotorRechts.MotorSensorCurrent);
    #endif
  }
  
  storeData();

  OCR2B = MotorRechts.ComputeMotorOutputs();
  OCR2A = MotorLinks.ComputeMotorOutputs();
}
