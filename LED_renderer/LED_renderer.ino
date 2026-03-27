#include <Arduino.h>
#include <FastLED.h>

HardwareSerial SpectrumSerial(2);

#define RXD2 25
#define TXD2 17 //Nepoužívá se

#define LED_PIN 26
#define NUM_LEDS 12
#define NUM_BANDS 12

CRGB leds[NUM_LEDS];

uint8_t bands[NUM_BANDS];
float smoothBands[NUM_BANDS];

enum ParseState {
  WAIT_START,
  READ_DATA,
  WAIT_END
};

ParseState state = WAIT_START;

uint8_t rxBuffer[NUM_BANDS];
int rxIndex = 0;

void renderBands() {

  for (int i = 0; i < NUM_BANDS; i++) {

    smoothBands[i] = smoothBands[i] * 0.7 + bands[i] * 0.3;

    uint8_t v = (uint8_t)smoothBands[i];

    uint8_t hue = map(i, 0, NUM_BANDS - 1, 0, 160);

    leds[i] = CHSV(hue, 255, v);
  }

  FastLED.show();
}

void processByte(uint8_t b) {

  switch (state) {

    case WAIT_START:

      if (b == 0xAA) {
        rxIndex = 0;
        state = READ_DATA;
      }

      break;

    case READ_DATA:

      rxBuffer[rxIndex++] = b;

      if (rxIndex >= NUM_BANDS)
        state = WAIT_END;

      break;

    case WAIT_END:

      if (b == 0x55) {

        memcpy(bands, rxBuffer, NUM_BANDS);

        renderBands();
      }

      state = WAIT_START;

      break;
  }
}

void setup() {

  Serial.begin(115200);

  SpectrumSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

  FastLED.setBrightness(150);

  for (int i = 0; i < NUM_BANDS; i++) {
    smoothBands[i] = 0;
  }

  Serial.println("ESP32 B - WS2812 renderer");
}

void loop() {

  while (SpectrumSerial.available()) {

    uint8_t b = SpectrumSerial.read();

    processByte(b);
  }
}