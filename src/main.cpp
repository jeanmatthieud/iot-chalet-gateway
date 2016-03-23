#include <Arduino.h>

#include <RF24.h>
#include <SPI.h>

#define LED_S1_BLUE A0
#define LED_S1_GREEN A1
#define LED_S1_RED A2

#define LED_S2_BLUE A3
#define LED_S2_GREEN A4
#define LED_S2_RED A5

#define MAX_SENSORS 2

/////////

typedef struct {
  unsigned long messageReceivedTime;
  unsigned long messageReceivedCounter;
  bool messageReceivedState;
} LedStatus;

typedef struct {
  uint8_t address[5];
  int minValue;
  int maxValue;
  LedStatus ledStatus;
} SensorNode;

//////////

void setMessageReceived(SensorNode &sensor);
void processLeds(unsigned long totalTime);
void processLedMessageReceived(unsigned long totalTime);

//////////

RF24 radio(7, 8);

uint8_t gatewayAddress[] = { 0x00, 0xA1, 0xB2, 0xC3, 0xD4 };

byte buffer[32];

SensorNode sensorNodes[MAX_SENSORS];

void setup() {
  Serial.begin(115200);

  // Analog output
  pinMode(LED_S1_RED, OUTPUT);
  pinMode(LED_S1_GREEN, OUTPUT);
  pinMode(LED_S1_BLUE, OUTPUT);
  pinMode(LED_S2_RED, OUTPUT);
  pinMode(LED_S2_GREEN, OUTPUT);
  pinMode(LED_S2_BLUE, OUTPUT);

  // Setup and configure radio
  radio.begin();
  radio.enableDynamicPayloads();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setAutoAck(true);

  radio.openReadingPipe(1, gatewayAddress);
  radio.startListening();

  //radio.printDetails();
}

void loop() {
  byte pipeNo;
  while(radio.available(&pipeNo)) {
    int payloadSize = radio.getDynamicPayloadSize();
    if(payloadSize < 1) {
      Serial.println("Corrupted payload");
      return;
    }
    radio.read( &buffer, sizeof(buffer) );

    Serial.print(F("Got value "));
    Serial.println(buffer[0]); // TODO

    setMessageReceived(sensorNodes[0]);
 }

 const unsigned long totalTime = millis();
 processLeds(totalTime);
}

void processLeds(unsigned long totalTime) {

  processLedMessageReceived(totalTime);
}

void processLedMessageReceived(unsigned long totalTime) {
  for(unsigned int i = 0; i < MAX_SENSORS; i++) {
    if(sensorNodes[i].ledStatus.messageReceivedCounter > 0) {
      unsigned long cycleTime = totalTime - sensorNodes[i].ledStatus.messageReceivedTime;

      if (cycleTime < 100) {
        digitalWrite(LED_S1_BLUE, HIGH);
        sensorNodes[i].ledStatus.messageReceivedState = true;
      }
      else if(sensorNodes[i].ledStatus.messageReceivedState) {
        digitalWrite(LED_S1_BLUE, LOW);
        sensorNodes[i].ledStatus.messageReceivedCounter -= 1;
        sensorNodes[i].ledStatus.messageReceivedState = false;
        sensorNodes[i].ledStatus.messageReceivedTime = totalTime + 100;
      }
    }
  }
}

void setMessageReceived(SensorNode &sensor) {
  sensor.ledStatus.messageReceivedTime = millis();
  sensor.ledStatus.messageReceivedCounter = 5;
  sensor.ledStatus.messageReceivedState = false;
}

void setColor(int red, int green, int blue) {
  analogWrite(LED_S1_RED, red);
  analogWrite(LED_S1_GREEN, green);
  analogWrite(LED_S1_BLUE, blue);
}
