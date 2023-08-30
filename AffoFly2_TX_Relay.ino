#include "printf.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define DEBUG

#define LED_PIN           5
#define CPPM_PIN          6
#define NRF_CE_PIN        9
#define NRF_CSN_PIN       10
#define RADIO_PIPE              0xE8E8F0F0E1LL
#define RADIO_CHANNEL           125
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
uint16_t PPM[CHANNEL_COUNT];

uint32_t currentTime = 0;
uint16_t sendRadioInterval = 0;
uint32_t lastSendRadioMillis = 0;

uint16_t loopCount = 0;
uint16_t radioCount = 0;
uint32_t ppmCount = 0;

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("System starting...");
#endif
  Radio_init();
  CPPM_init();
}

void loop() {
  currentTime = millis();
  if (currentTime - lastSendRadioMillis >= sendRadioInterval) {
    lastSendRadioMillis = currentTime;
    Radio_output();
  }
  loopCount++;
}

void CPPM_init() {
#ifdef DEBUG
  Serial.print("Initialising CPPM......");
#endif
  pinMode(CPPM_PIN, OUTPUT);
  PORTD = PORTD & ~B01000000;  //Set CPPM to low
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  OCR1A = 100;  // compare match register (not very important, sets the timeout for the first interrupt)
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
#ifdef DEBUG
  Serial.println("Done");
#endif
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

void Radio_output() {
  //uint32_t before = millis();
  radio.write(&controlData, sizeof(ControlData));
  //  uint32_t after = millis();
  //  if(after - before > 10){
  //    //TODO: once radio stuck, it will not revived by itself, intervention required.
  //    Serial.print("Radio Slow! took: ");   Serial.println(after - before);
  //  }
  radioCount++;
}

ISR(TIMER1_COMPA_vect) {
  PPM[0] = controlData.Throttle;
  PPM[1] = controlData.Yaw;
  PPM[2] = controlData.Pitch;
  PPM[3] = controlData.Roll;
  PPM[4] = controlData.Aux1;
  PPM[5] = controlData.Aux2;
  PPM[6] = controlData.Aux3;
  PPM[7] = controlData.Aux4;

  static boolean state = true;
  TCNT1 = 0;
  if ( state ) {
    //end pulse
    PORTD = PORTD & ~B01000000; // turn pin 6 off.
    OCR1A = PPM_PULSE_LENGTH * CLOCK_MULTIPLIER;
    state = false;
  } else {
    //start pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    PORTD = PORTD | B01000000; // turn pin 6 on.
    state = true;

    if (cur_chan_numb >= CHANNEL_COUNT) {
      cur_chan_numb = 0;
      calc_rest += PPM_PULSE_LENGTH;
      OCR1A = (PPM_FRAME_LENGTH - calc_rest) * CLOCK_MULTIPLIER;
      calc_rest = 0;
    }
    else {
      OCR1A = (PPM[cur_chan_numb] - 10 - PPM_PULSE_LENGTH) * CLOCK_MULTIPLIER;
      calc_rest += PPM[cur_chan_numb] - 10;
      cur_chan_numb++;
    }
  }

  ppmCount++;
}