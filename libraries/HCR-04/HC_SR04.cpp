#include "HC_SR04.h"
#include "Arduino.h"

//HC_SR04 *HC_SR04::_instance=NULL;
HC_SR04 *HC_SR04::_instance(NULL);

HC_SR04::HC_SR04(int trigger1, int trigger2, int trigger3, int trigger4, int trigger5,
                 int trigger6, int trigger7, int trigger8, int trigger9, int trigger10,
                 int echo, int interrupt, int max_dist) : _echo(echo), _int(interrupt),
                 _max(max_dist), _finished(true){

  _trigger[0] = trigger1;
  _trigger[1] = trigger2;
  _trigger[2] = trigger3;
  _trigger[3] = trigger4;
  _trigger[4] = trigger5;
  _trigger[5] = trigger6;
  _trigger[6] = trigger7;
  _trigger[7] = trigger8;
  _trigger[8] = trigger9;
  _trigger[9] = trigger10;

  if(_instance==0){
    _instance=this;
  }
}

void HC_SR04::begin(bool units){
  for (int a = 0; a < 10; a++) {
    pinMode(_trigger[a], OUTPUT);
    digitalWrite(_trigger[a], LOW);
  }
  pinMode(_echo, INPUT);
  attachInterrupt(_int, _echo_isr, CHANGE);
  _maxtime = _max * ((units)?58:148);
}

void HC_SR04::start(int trigger){
  HC_SR04* _this=HC_SR04::instance();
  _finished=false;
  digitalWrite(_trigger[trigger], HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigger[trigger], LOW);
  _this->_startDelay=micros();
}

unsigned int HC_SR04::getRange(bool units){
  return (_end-_start)/((units)?58:148);
}

int HC_SR04::isFinished(){
  HC_SR04* _this=HC_SR04::instance();
  if (not _this->_finished){
    if (_this->_measurementongoing){
      _this->_Checkend=micros();
      if(int(_this->_Checkend - _this->_start) > _maxtime){
        _this->_finished = true;
        _this->_measurementongoing = false;
        return 2;
      }
    }
    else{
      _this->_endDelay=micros();
      if ((_this->_endDelay - _this->_startDelay) > 5000){
        _this->_finished = true;
        _this->_measurementongoing = false;
        return 3;
      }
      else{
        return 0;
      }
    }
  }
  else{
    return 1;
  }
  return 0;
}

void HC_SR04::_echo_isr(){
  HC_SR04* _this=HC_SR04::instance();
  switch(digitalRead(_this->_echo)){
    case HIGH:
      if (not _this->_finished){
        _this->_start=micros();
        _this->_measurementongoing = true;
      }
      break;
    case LOW:
      if (_this->_measurementongoing){
        _this->_end=micros();
        _this->_measurementongoing = false;
        _this->_finished=true;
      }
      break;
  }
}
