/*
    MotorDriver.cpp - Library for Motors with double H-Bridge, Sensored, Enable Pin & CurrenSensorPin
*/

#include "Arduino.h"
#include "MotorDriver.h"
#include "PID_v1.h"

MotorDriver::MotorDriver(
  double HighPhasepercentage,
  double Kp_S,
  double Ki_S,
  double Kd_S,
  double Kp_C,
  double Ki_C,
  double Kd_C,
  double* MotorSpeedInput,
  double* MotorMaxCurrent,
  bool* MotorError,
  bool* MotorDirection,
  bool* MotorEnabled,
  byte MotorEnablePin,  byte MotorPWMPin,
  byte MotorOutputAPin, byte MotorOutputBPin,
  byte MotorCurrentSensorPin, byte MotorRotationSensorPin):
  _MotorSpeedPID(&MotorSensorSpeed, &_MotorSpeedOutput, MotorSpeedInput, Kp_S, Ki_S, Kd_S, DIRECT),
  _MotorCurrentPID(&MotorSensorCurrent, &_MotorCurrentLimiter, MotorMaxCurrent, Kp_C, Ki_C, Kd_C, DIRECT)
  {
    //Vraiables with hard Links
    _MotorSpeedInput = MotorSpeedInput;
    _MotorMaxCurrent = MotorMaxCurrent;
    _MotorError = MotorError;
    _MotorDirection = MotorDirection;
    _MotorEnabled = MotorEnabled;

    //Variables internal
    _MotorEnablePin = MotorEnablePin;
    _MotorPWMPin = MotorPWMPin;
    _MotorOutputAPin = MotorOutputAPin;
    _MotorOutputBPin = MotorOutputBPin;
    _MotorCurrentSensorPin = MotorCurrentSensorPin;
    _MotorRotationSensorPin = MotorRotationSensorPin;

    //Kp_S=2;
    //Ki_S=12;
    //Kd_S=0.02;

    //Kp_C=2;
    //Ki_C=3;
    //Kd_C=0;

    //_MotorSpeedPID.SetTunings(Kp_S, Ki_S, Kd_S);
    //_MotorCurrentPID.SetTunings(Kp_C, Ki_C, Kd_C);

    MotorDriver::CalcCorrectionFactors(HighPhasepercentage);

  // set the digital pin as output:
  pinMode(_MotorEnablePin,  OUTPUT);
  pinMode(_MotorPWMPin,     OUTPUT);
  pinMode(_MotorOutputAPin,    OUTPUT);
  pinMode(_MotorOutputBPin,    OUTPUT);

  pinMode(_MotorCurrentSensorPin,   INPUT);
  pinMode(_MotorRotationSensorPin,  INPUT);

  digitalWrite(_MotorRotationSensorPin, LOW); // Pulldown Rotation pin for cabel break seafty

  //set all outputs low
  digitalWrite(_MotorEnablePin, LOW);
  digitalWrite(_MotorOutputAPin,   LOW);
  digitalWrite(_MotorOutputBPin,   LOW);

  _MotorSpeedPID.SetSampleTime(10);
  _MotorCurrentPID.SetSampleTime(100);

  _MotorCurrentPID.SetMode(AUTOMATIC);
  //_MotorCurrentPID.SetMode(MANUAL);

  _MotorRotationPinStatus = digitalRead(_MotorRotationSensorPin);
}

void MotorDriver::CalcCorrectionFactors(double _HighPhasepercentage){
  _SpeedCorrectionFactorHigh = 50.0 / _HighPhasepercentage;
  _SpeedCorrectionFactorLow  = 50.0 / (100.0 - _HighPhasepercentage);
}

void MotorDriver::GetMotorCurrent(){
  // subtract the last reading:
  _MotorCurrenttotal = _MotorCurrenttotal - _MotorCurrentRead[_MotorCurrentindex];
  // read from the sensor:
  _MotorCurrentRead[_MotorCurrentindex] = analogRead(_MotorCurrentSensorPin);
  // add the reading to the total:
  _MotorCurrenttotal = _MotorCurrenttotal + _MotorCurrentRead[_MotorCurrentindex];
  // advance to the next position in the array:
  ++_MotorCurrentindex;
  if (_MotorCurrentindex >= _NumberOfReadingsCurrent){
    _MotorCurrentindex = 0;
  }
  MotorSensorCurrent = _MotorCurrenttotal / _NumberOfReadingsCurrent;
}

void MotorDriver::ReadMotorSensorSpeed(){
  _MotorSensorTimeNew = micros();

  if (_MotorSensorTimeNew >= _MotorSensorTimeOld){
    _MotorSensorTime = _MotorSensorTimeNew - _MotorSensorTimeOld;
  }
  else{
    _MotorSensorTime = (4294967295 - _MotorSensorTimeOld) + _MotorSensorTimeNew;
  }

  if (_MotorRotationPinStatus != digitalRead(_MotorRotationSensorPin)){
    _MotorRotationPinStatus = digitalRead(_MotorRotationSensorPin);
    _MotorSensorTimeOld = _MotorSensorTimeNew;
    MotorSensorSpeed = 1000000.0 / double(_MotorSensorTime);
    if (_MotorRotationPinStatus == 1){
      MotorSensorSpeed = MotorSensorSpeed * _SpeedCorrectionFactorHigh;
    }
    else{
      MotorSensorSpeed = MotorSensorSpeed * _SpeedCorrectionFactorLow;
    }
    _SpeedIntegralMotor = 0;
    ++MotorMoved;
  }
  else{
    if (_MotorSensorTime > 330000){
      _MotorSensorTimeOld = _MotorSensorTimeNew;
      MotorSensorSpeed = 0;
      if (*_MotorEnabled){
        if ((_MotorSpeedOutput > 30.0) && (MotorSensorCurrent > 20.0)){
          _SpeedIntegralMotor = _SpeedIntegralMotor + int(_MotorSpeedOutput);
          if (_SpeedIntegralMotor > 500){
            *_MotorError = true;
            *_MotorEnabled = false;
          }
        }
        else if ((_MotorSpeedOutput > 30.0) && (MotorSensorCurrent < 20.0)){
          *_MotorError = true;
          *_MotorEnabled = false;
        }
      }
      else{
        _SpeedIntegralMotor = 0;
      }
    }
  }
}

void MotorDriver::SetupMotorPins(){
  if ((*_MotorEnabled != digitalRead(_MotorEnablePin)) || (*_MotorError)) {
    if ((*_MotorEnabled) and (not *_MotorError)){
      //turn the PID on
      _MotorSpeedPID.SetMode(AUTOMATIC);
    }
    else{
      digitalWrite(_MotorOutputAPin, LOW);
      digitalWrite(_MotorOutputBPin, LOW);
      //turn the PID off
      _MotorSpeedPID.SetMode(MANUAL);
      *_MotorEnabled = false;
    }
    digitalWrite(_MotorEnablePin, *_MotorEnabled);
    _MotorSpeedOutput = 0;
  }
  if (*_MotorEnabled){
    if (*_MotorDirection){
      digitalWrite(_MotorOutputAPin, LOW);
      digitalWrite(_MotorOutputBPin, HIGH);
    }
    else{
      digitalWrite(_MotorOutputAPin, HIGH);
      digitalWrite(_MotorOutputBPin, LOW);
    }
  }
}

byte MotorDriver::ComputeMotorOutputs(){

  MotorDriver::SetupMotorPins();

  MotorDriver::GetMotorCurrent();
  MotorDriver::ReadMotorSensorSpeed();

  _MotorCurrentPID.Compute();
  _MotorSpeedPID.SetOutputLimits(0, _MotorCurrentLimiter);
  _MotorSpeedPID.Compute();
  return _MotorSpeedOutput;
}
