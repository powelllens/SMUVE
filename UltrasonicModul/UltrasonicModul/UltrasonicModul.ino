/*

*/

#include <Wire.h>
#include <avr/wdt.h>

#define Debug 1

//Constant Variables
// Motorpins 1
const int Ultrasonictrigger[10] = {9, 8, 7, 6, 5, 4, 3, 2, A0, A1};
const int Ultrasonicreciever = A2;

//Runtime Variables
int Ultrasonicsensor[10];

long duration;
int distance[10];


/* Wire variables */
#define SLAVE_ADDRESS   0x09
#define REG_MAP_SIZE    11
#define MAX_SENT_BYTES  3

/********* Global  Variables  ***********/
bool newDataAvailable;
byte zeroA, zeroB;
byte zeroAData, zeroBData;

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
  pinMode(12, OUTPUT);
  wdt_enable(WDTO_1S);

  Serial.begin(115200);

  Wire.begin(SLAVE_ADDRESS);    // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent);
  Wire.setClock(400000);

  // set the digital pin as output:
  for (int i = 0; i <= 9; i++) {
    pinMode(Ultrasonictrigger[i],  OUTPUT);
    digitalWrite(Ultrasonictrigger[i], LOW);
  }
  pinMode(Ultrasonicreciever,  INPUT);
}

void loop() {
  // Sets the trigPin on HIGH state for 10 micro seconds
  for (int i = 0; i <= 9; i++) {
    digitalWrite(Ultrasonictrigger[i], HIGH);
    delayMicroseconds(5);
    digitalWrite(Ultrasonictrigger[i], LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(Ultrasonicreciever, HIGH, 15000);
    if (duration == 0 || duration > 9412) {
      duration = 9412;
    }
    // Calculating the distance
    distance[i] = (duration * 0.034) / 2;
#if Debug
    if (i == 9) {
      Serial.print(distance[i]);
      Serial.print(" - ");
    }
    else {
      Serial.println(distance[i]);
    }
#endif
  }

  storeData();
  newDataAvailable = 1;
}
