#include <Arduino.h>

#include <RF24.h>
#include <SPI.h>

#define LED_S1_BLUE A0
#define LED_S1_GREEN A1
#define LED_S1_RED A2

#define LED_S2_BLUE A3
#define LED_S2_GREEN A4
#define LED_S2_RED A5

RF24 radio(7, 8);

uint32_t displayTimer = 0;

uint8_t gatewayAddress[] = { 0x00, 0xA1, 0xB2, 0xC3, 0xD4 };
uint8_t nodeAddress[] = { 0x01, 0xA1, 0xB2, 0xC3, 0xD4 };
byte counter = 1;

byte buffer[32];

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

  //radio.openWritingPipe(nodeAddress);
  radio.openReadingPipe(1, gatewayAddress);
  radio.startListening();

  radio.printDetails();
}

void loop() {

  //analogWrite(LED_S2_RED, 255);
  //analogWrite(LED_S2_GREEN, 255);
  //analogWrite(LED_S2_BLUE, 255);

  byte pipeNo;
  while(radio.available(&pipeNo)) {
    int payloadSize = radio.getDynamicPayloadSize();
    if(payloadSize < 1) {
      Serial.println("Corrupted payload");
      return;
    }
    radio.read( &buffer, sizeof(buffer) );

    Serial.print(F("Got value "));
    Serial.println(buffer[0]);

    analogWrite(LED_S1_BLUE, 255);
    delay(50);
    analogWrite(LED_S1_BLUE, 0);
 }
}
