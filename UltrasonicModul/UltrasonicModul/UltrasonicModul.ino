/*

*/

#include <Wire.h>
#include <avr/wdt.h>
#include "HC_SR04.h"

#define debug 0

#define ECHO_PIN 2
#define ECHO_INT 0

//HC_SR04 sensor(A1, A0, A2, 3, 4, 5, 6, 7, 8, 9, ECHO_PIN, ECHO_INT, 150);
HC_SR04 sensor(A1, A0, A2, 3, 4, 5, 6, 7, 8, 9, ECHO_PIN, ECHO_INT, 150);
const byte Sequenz[10] = {3, 9, 1, 0, 7, 5, 4, 2, 6, 8};

byte distance[10];
byte i = 0;
int DistanceBuffer;
byte SensorState;

/* Wire variables */
#define SLAVE_ADDRESS   0x09
#define REG_MAP_SIZE    11
#define MAX_SENT_BYTES  3

/********* Global  Variables  ***********/
bool newDataAvailable;

byte registerMap[REG_MAP_SIZE];
byte registerMapTemp[REG_MAP_SIZE - 1];
byte receivedCommands[MAX_SENT_BYTES];

void receiveEvent(int bytesReceived) {
  wdt_reset();
  for (int a = 0; a < bytesReceived; a++) {
    if ( a < MAX_SENT_BYTES) {
      receivedCommands[a] = Wire.read();
    }
    else {
      Wire.read();  // if we receive more data then allowed just throw it away
    }
  }
  if (bytesReceived == 1 && (receivedCommands[0] < REG_MAP_SIZE)) {
    return;
  }
  if (bytesReceived == 1 && (receivedCommands[0] >= REG_MAP_SIZE)) {
    receivedCommands[0] = 0x00;
    return;
  }
}

void requestEvent() {
  wdt_reset();
  if (newDataAvailable) {
    for (int c = 0; c < (REG_MAP_SIZE - 1); c++) {
      registerMap[c] = registerMapTemp[c];
    }
    newDataAvailable = 0;
  }
  //Set the buffer up to send all 14 bytes of data
  Wire.write(registerMap + receivedCommands[0], REG_MAP_SIZE);
}

void storeData() {
  for (int i = 0; i < 10; i++) {
    registerMapTemp[i] = distance[i];  //increment pointer to store each byte
  }
}

void setup() {
  wdt_enable(WDTO_1S);
  Serial.begin(115200);

  sensor.begin();

  Wire.begin(SLAVE_ADDRESS);    // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent);
  Wire.setClock(400000);

  sensor.start(Sequenz[i]);

}

void loop() {

  SensorState = sensor.isFinished();
  if (SensorState > 0) {
    DistanceBuffer = sensor.getRange();
    if (SensorState > 1) {
      DistanceBuffer = 150;
    }
    distance[Sequenz[i]] = DistanceBuffer;
    i++;
    if (i > 9) {
      i = 0;
#if debug
      for (int c = 0; c < 9; c++) {
        if (distance[c] < 10) {
          Serial.print("00");
        }
        else if (distance[c] < 100) {
          Serial.print("0");
        }
        Serial.print(distance[c]);
        Serial.print(" ");
      }
      Serial.println(distance[9]);
#endif
    }
    sensor.start(Sequenz[i]);
  }
  delay(1);

  storeData();
  newDataAvailable = 1;
}

