#include <Arduino.h>
#include <math.h>

#include <RF24.h>
#include <SPI.h>

// - Pins of the first RGB LED
#define LED_S1_BLUE A0
#define LED_S1_GREEN 5
#define LED_S1_RED 6

// - Pins of the second RGB LED
#define LED_S2_BLUE A1
#define LED_S2_GREEN 9
#define LED_S2_RED 10

//////////

// - Number of sensors
#define MAX_SENSORS 2

// - Delay and blink count for the blinking led (message received)
#define LED_MSG_RECEIVED_BLINK_DELAY 75
#define LED_MSG_RECEIVED_BLINK_COUNT 3
// - Power coefficient for LEDS
#define LED_POWER_COEFF 0.1f

// - RF messages payload size
#define PAYLOAD_SIZE 3

/////////

// - Datastore for blinking led (message received)
typedef struct {
  // - Last 'off' state in ms
  unsigned long lastBlinkEnd;
  // - How many more time the led has to blink
  unsigned short blinkLeft;
  // - Is the led on ?
  bool active;
  // - lED pin
  int pin;
} LedStatus;

// - Datastore for sensors
typedef struct {
  // - Address of the sensor
  uint8_t address;
  // - Current measured value (in cm)
  int currentValue;
  // - Lower value measured since power-on
  int minValue;
  // - Higher value measured since power-on
  int maxValue;
  // - Blinking led data
  LedStatus ledStatus;
  // - Red LED pin
  int pinRed;
  // - Green LED pin
  int pinGreen;
} SensorNode;

// - Message definition
typedef struct {
  // - Address of the device
  uint8_t address;
  // - Value of the sensor in centimeters
  unsigned short value;
} Message;

//////////

void setMessageReceived(SensorNode &sensor);
void saveValue(SensorNode &sensor, int value);
void processLeds(unsigned long currentTime);
void processLedMessageReceived(unsigned long currentTime);
void processLedColor(unsigned long currentTime);

//////////

// - RF interface
RF24 radio(7, 8);
// - Sensor nodes datastores
SensorNode sensorNodes[MAX_SENSORS];

// - Gateway address
uint8_t gatewayAddress[] = { 0x00, 0xA1, 0xB2, 0xC3, 0xD4 };

// - Buffer to store received messages
byte buffer[32];

void setup() {
  // - Serial init
  Serial.begin(115200);

  // - Analog / Digital outputs
  pinMode(LED_S1_RED, OUTPUT);
  pinMode(LED_S1_GREEN, OUTPUT);
  pinMode(LED_S1_BLUE, OUTPUT);
  pinMode(LED_S2_RED, OUTPUT);
  pinMode(LED_S2_GREEN, OUTPUT);
  pinMode(LED_S2_BLUE, OUTPUT);

  // - Init sensors datastore
  sensorNodes[0].ledStatus.pin = LED_S1_BLUE;
  sensorNodes[0].pinRed = LED_S1_RED;
  sensorNodes[0].pinGreen = LED_S1_GREEN;
  sensorNodes[1].ledStatus.pin = LED_S2_BLUE;
  sensorNodes[1].pinRed = LED_S2_RED;
  sensorNodes[1].pinGreen = LED_S2_GREEN;

  // - Setup and configure radio
  radio.begin();
  //radio.enableDynamicPayloads();
  radio.setPayloadSize(PAYLOAD_SIZE);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setAutoAck(true);

  // - Listen for messages
  radio.openReadingPipe(1, gatewayAddress);
  radio.startListening();
}

void loop() {
  byte pipeNo;
  while(radio.available(&pipeNo)) {
    int payloadSize = radio.getDynamicPayloadSize();
    if(payloadSize < 1) { // - TODO : check if differs from the static payload size
      Serial.println("Corrupted payload");
      return;
    }
    radio.read( &buffer, sizeof(buffer) );

    // - Message buffer to message mapping
    Message msg;
    msg = *(Message *)buffer;

    // - Get which LED datastore should be edited, based on the message address
    SensorNode* pSensorNode = NULL;
    if(msg.address == 0x01) {
      pSensorNode = &sensorNodes[0];
    } else if(msg.address == 0x02) {
      pSensorNode = &sensorNodes[1];
    } else {
      Serial.println("Address not known !!");
      return;
    }

    // - Save the value received
    saveValue(*pSensorNode, msg.value);
    // - Update received status
    setMessageReceived(*pSensorNode);
  }

  // - Update LED state (without using "delay()")
  const unsigned long currentTime = millis();
  processLeds(currentTime);
}

void processLeds(unsigned long currentTime) {
  processLedMessageReceived(currentTime);
  processLedColor(currentTime);
}

void processLedMessageReceived(unsigned long currentTime) {
  // - Blinks the blue LED if necessary, without using "delay()"
  for(unsigned int i = 0; i < MAX_SENSORS; i++) {
    if(sensorNodes[i].ledStatus.blinkLeft > 0) {
      unsigned long cycleTime = currentTime - sensorNodes[i].ledStatus.lastBlinkEnd;

      if (cycleTime < LED_MSG_RECEIVED_BLINK_DELAY) {
        sensorNodes[i].ledStatus.active = true;
      } else if(sensorNodes[i].ledStatus.active) {
        sensorNodes[i].ledStatus.blinkLeft -= 1;
        sensorNodes[i].ledStatus.active = false;
        sensorNodes[i].ledStatus.lastBlinkEnd = currentTime + LED_MSG_RECEIVED_BLINK_DELAY;
      }
      digitalWrite(sensorNodes[i].ledStatus.pin, sensorNodes[i].ledStatus.active ? HIGH : LOW);
    }
  }
}

void processLedColor(unsigned long currentTime) {
  // - Update red / green LEDs to create a scale between the min and max value
  for(unsigned int i = 0; i < MAX_SENSORS; i++) {
    if(sensorNodes[i].currentValue != 0 && (sensorNodes[i].maxValue - sensorNodes[i].minValue) > 0) {
      int currentPercent = 100 * (sensorNodes[i].currentValue - sensorNodes[i].minValue) / (sensorNodes[i].maxValue - sensorNodes[i].minValue);
      int redValue = (255 * currentPercent) / 100;
      int greenValue = (255 * (100 - currentPercent)) / 100;

      // - A LED COEFF is applied to avoid blindness after looking at the LEDs
      analogWrite(sensorNodes[i].pinRed, (int)round(redValue * LED_POWER_COEFF));
      analogWrite(sensorNodes[i].pinGreen, (int)round(greenValue * LED_POWER_COEFF));
    }
  }
}

void setMessageReceived(SensorNode &sensor) {
  sensor.ledStatus.lastBlinkEnd = millis();
  sensor.ledStatus.blinkLeft = LED_MSG_RECEIVED_BLINK_COUNT;
  sensor.ledStatus.active = false;
}

void saveValue(SensorNode &sensor, int value) {
  Serial.print(F("Got value "));
  Serial.println(value);

  // - "0" is received when the sensor max distance is reached. Skipping...
  if(value == 0) {
    return;
  }

  sensor.currentValue = value;
  if(value > sensor.maxValue) {
    sensor.maxValue = value;
  }
  if(value < sensor.minValue) {
    sensor.minValue = value;
  }
}
