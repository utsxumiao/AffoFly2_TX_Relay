#include "printf.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <PPMReader.h>

#define DEBUG
#define SHOW_OUPUT
#define SHOW_STATS

#define LED_PIN           5
#define CPPM_PIN          3
#define NRF_CE_PIN        9
#define NRF_CSN_PIN       10
#define RADIO_PIPE        0xE8E8F0F0E1LL
#define RADIO_CHANNEL     125
#define RADIO_TOKEN       "5734897589"
#define CLOCK_MULTIPLIER  1       // 1 for 8MHZ, 2 for 16MHZ
#define PPM_FRAME_LENGTH  20000
#define PPM_PULSE_LENGTH  400
#define CHANNEL_COUNT     8

struct ControlData {
  uint32_t Token;
  uint16_t Throttle;
  uint16_t Yaw;
  uint16_t Pitch;
  uint16_t Roll;
  uint16_t Aux1;
  uint16_t Aux2;
  uint16_t Aux3;
  uint16_t Aux4;
};

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
ControlData controlData;
PPMReader ppm(CPPM_PIN, CHANNEL_COUNT);

uint32_t currentTime = 0;
uint16_t sendRadioInterval = 50;
uint32_t lastSendRadioMillis = 0;
uint16_t showStatsInterval = 1000;
uint32_t lastShowStatsMillis = 0;
uint16_t loopCount = 0;
uint16_t radioCount = 0;

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("System starting...");
#endif
  Radio_init();
  controlData.Token = RADIO_TOKEN;
}

void loop() {
  currentTime = millis();
  if (currentTime - lastSendRadioMillis >= sendRadioInterval) {
    lastSendRadioMillis = currentTime;
    PPM_Read();
    Radio_output();
    radioCount++;
  }

  if(currentTime - lastShowStatsMillis >= showStatsInterval) {
    lastShowStatsMillis = currentTime;
    Show_Stats();
  }

  loopCount++;
}

void Radio_init() {
#ifdef DEBUG
  Serial.print("Initialising Radio......");
#endif
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(false);
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(RADIO_PIPE);
  radio.stopListening();
#ifdef DEBUG
  radio.printDetails();
  Serial.println("Done");
#endif
}

void PPM_Read() {
  controlData.Throttle = ppm.latestValidChannelValue(1, 1000);
  controlData.Yaw      = ppm.latestValidChannelValue(2, 1500);
  controlData.Pitch    = ppm.latestValidChannelValue(3, 1500);
  controlData.Roll     = ppm.latestValidChannelValue(4, 1500);
  controlData.Aux1     = ppm.latestValidChannelValue(5, 1000);
  controlData.Aux2     = ppm.latestValidChannelValue(6, 1000);
  controlData.Aux3     = ppm.latestValidChannelValue(7, 1000);
  controlData.Aux4     = ppm.latestValidChannelValue(8, 1000);

#ifdef SHOW_OUPUT
  Serial.print("Throttle: ");     Serial.print(controlData.Throttle);   Serial.print("    ");
  Serial.print("Yaw: ");          Serial.print(controlData.Yaw);        Serial.print("    ");
  Serial.print("Pitch: ");        Serial.print(controlData.Pitch);      Serial.print("    ");
  Serial.print("Roll: ");         Serial.print(controlData.Roll);       Serial.print("    ");
  Serial.print("Aux1: ");         Serial.print(controlData.Aux1);       Serial.print("    ");
  Serial.print("Aux2: ");         Serial.print(controlData.Aux2);       Serial.print("    ");
  Serial.print("Aux3: ");         Serial.print(controlData.Aux3);       Serial.print("    ");
  Serial.print("Aux4: ");         Serial.print(controlData.Aux4);       Serial.print("    ");
  Serial.println("");
#endif
}

void Radio_output() {
  radio.write(&controlData, sizeof(ControlData));
}

void Show_Stats() {
  Serial.print("Loop: ");       Serial.print(loopCount);            Serial.print("    ");
  Serial.print("Radio: ");      Serial.print(radioCount);           Serial.print("    ");
  Serial.println();
  loopCount = 0;
  radioCount = 0;
}