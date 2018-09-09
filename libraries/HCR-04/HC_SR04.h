#ifndef HC_SR04_H
#define HC_SR04_H

#include <Arduino.h>

#define CM true
#define INCH false

class HC_SR04 {
  public:
    HC_SR04(int trigger1, int trigger2, int trigger3, int trigger4, int trigger5,
            int trigger6, int trigger7, int trigger8, int trigger9, int trigger10,
            int echo, int interrupt, int max_dist=150);

    void begin(bool units=CM);
    void start(int trigger);
    int isFinished();
    unsigned int getRange(bool units=CM);
    static HC_SR04* instance(){ return _instance; }

  private:
    static void _echo_isr();

    int _echo, _int, _max, _maxtime;
    int _trigger[10];
    volatile unsigned long _start, _end, _startDelay, _endDelay, _Checkend;
    volatile bool _finished, _measurementongoing;
    static HC_SR04* _instance;
};

#endif
