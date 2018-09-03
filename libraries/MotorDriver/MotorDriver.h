/*
    MotorDriver.cpp - Library for Motors with double H-Bridge, Sensored, Enable Pin & CurrenSensorPin
*/

#ifndef MotorDriver_h
#define MotorDriver_h

#include "Arduino.h"
#include "PID_v1.h"

const byte _NumberOfReadingsCurrent = 5;

class MotorDriver
{
  public:
    //Motor Current
    double MotorSensorCurrent;
    //Motor Speed
    double MotorSensorSpeed;
    //Motor moved
    word MotorMoved;

    MotorDriver(double,
    double, double, double,
    double, double, double,
    double*, double*,
    bool*, bool*, bool*,
    byte, byte, byte, byte, byte, byte);

    byte ComputeMotorOutputs();

    void CalcCorrectionFactors(double);

    //double *_MotorSpeedInput;
    //double _MotorSpeedOutput;
    //word _SpeedIntegralMotor;
  private:
    // I/O's
    double *_MotorSpeedInput;
    double *_MotorMaxCurrent;
    bool *_MotorError;
    bool *_MotorDirection;
    bool *_MotorEnabled;

    //PID for speed
    double Kp_S, Ki_S, Kd_S;
    double _MotorSpeedOutput;

    PID _MotorSpeedPID;
    double _SpeedCorrectionFactorHigh;
    double _SpeedCorrectionFactorLow;

    //PID for Current
    PID _MotorCurrentPID;
    double Kp_C, Ki_C, Kd_C;
    double _MotorCurrentLimiter;

    //All Pins
    byte _MotorEnablePin;
    byte _MotorPWMPin;
    byte _MotorOutputAPin;
    byte _MotorOutputBPin;
    byte _MotorCurrentSensorPin;
    byte _MotorRotationSensorPin;


    void GetMotorCurrent();

    int _MotorCurrenttotal;
    int _MotorCurrentRead[_NumberOfReadingsCurrent];
    byte _MotorCurrentindex;


    void ReadMotorSensorSpeed();

    unsigned long _MotorSensorTimeNew;
    unsigned long _MotorSensorTimeOld;
    unsigned long _MotorSensorTime;
    word _SpeedIntegralMotor;
    bool _MotorRotationPinStatus;

    void SetupMotorPins();

};

#endif
