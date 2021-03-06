#ifndef Pins_Arduino_h
#define Pins_Arduino_h

// This file is a slightly modified version of:
// https://github.com/ArduCAM/ArduCAM_ESP8266_UNO/blob/master/variants/ArduCAM_ESP8266_UNO/pins_arduino.h
// to account for some slightly different pin mappings on the board I have.

#include <stdint.h>

#define EXTERNAL_NUM_INTERRUPTS 16
#define NUM_DIGITAL_PINS        40
#define NUM_ANALOG_INPUTS       16

#define analogInputToDigitalPin(p)  (((p)<20)?(esp32_adc2gpio[(p)]):-1)
#define digitalPinToInterrupt(p)    (((p)<40)?(p):-1)
#define digitalPinHasPWM(p)         (p < 34)

static const uint8_t LED_BUILTIN = 2;
#define BUILTIN_LED  LED_BUILTIN // backward compatibility

static const uint8_t KEY_BUILTIN = 0;

static const uint8_t TX = 1;
static const uint8_t RX = 3;

static const uint8_t SDA = 21;
static const uint8_t SCL = 22;

static const uint8_t SS    = 5;
static const uint8_t MOSI  = 23;
static const uint8_t MISO  = 19;
static const uint8_t SCK   = 18;

static const uint8_t D0 = 3;
static const uint8_t D1 = 1;
static const uint8_t D2 = 13;
static const uint8_t D3 = 12;
static const uint8_t D4 = 14;
static const uint8_t D5 = 15;
static const uint8_t D6 = 25;
static const uint8_t D7 = 5;
static const uint8_t D8 = 2;
static const uint8_t D9 = 0;
static const uint8_t D10 = 4;
static const uint8_t D11 = 27;
static const uint8_t D12 = 26;

static const uint8_t S0 = 36;
static const uint8_t S1 = 39;
static const uint8_t S2 = 32;
static const uint8_t S3 = 33;
static const uint8_t S4 = 34;
static const uint8_t S5 = 35;

static const uint8_t A0 = 36;
static const uint8_t A3 = 39;
static const uint8_t A4 = 32;
static const uint8_t A5 = 33;
static const uint8_t A6 = 34;
static const uint8_t A7 = 35;
static const uint8_t A10 = 4;
static const uint8_t A11 = 0;
static const uint8_t A12 = 2;
static const uint8_t A13 = 15;
static const uint8_t A14 = 13;
static const uint8_t A15 = 12;
static const uint8_t A16 = 14;
static const uint8_t A17 = 27;
static const uint8_t A18 = 25;
static const uint8_t A19 = 26;

static const uint8_t T0 = 4;
static const uint8_t T1 = 0;
static const uint8_t T2 = 2;
static const uint8_t T3 = 15;
static const uint8_t T4 = 13;
static const uint8_t T5 = 12;
static const uint8_t T6 = 14;
static const uint8_t T7 = 27;
static const uint8_t T8 = 33;
static const uint8_t T9 = 32;

static const uint8_t DAC1 = 25;
static const uint8_t DAC2 = 26;


#endif /* Pins_Arduino_h */
