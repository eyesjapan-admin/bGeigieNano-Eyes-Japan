#ifndef _NANO_CONFIG_H_
#define _NANO_CONFIG_H_

//
// bGeigie Nano definitions
//

#define NANO_DEVICE_ID           210
#define NANO_VERSION         "1.1.4"
#define NANO_HEADER          "BNRDD"
#define NANO_HEADER_SENSOR  "BNXSTS"
#define NANO_CPM_FACTOR          334
#define NANO_BQM2_FACTOR          37

//
// Enable or Disable features
//

#define ENABLE_DEBUG             0
#define ENABLE_DIAGNOSTIC        0
#define ENABLE_SSD1306           1
#define ENABLE_SOFTGPS           1
#define ENABLE_OPENLOG           1
#define ENABLE_WAIT_GPS_FOR_LOG  1
#define ENABLE_LND_DEADTIME      1 // enable dead-time compensation for LND7317
#define ENABLE_SHT1              1 
#define ENABLE_GEIGIE_SWITCH     1

#if ENABLE_SSD1306 // high memory usage (avoid logs)
#undef ENABLE_DEBUG // disable debug log output
#endif

//
// Pins definition
//

  #warning NANO KIT with OLED screen used !
  #define OLED_SPI_MODE // SPI mode enabled
  #define OLED_CLK 7
  #define OLED_DATA 6
  #define OLED_DC 5
  #define OLED_CS 4
  #define OLED_RESET 3
  #define GPS_RX_PIN 8
  #define GPS_TX_PIN 9
  #define OPENLOG_RX_PIN 10
  #define OPENLOG_TX_PIN 11
  #define OPENLOG_RST_PIN 12
  #define GPS_LED_PIN 13
  #define LOGALARM_LED_PIN A4

// HardwareCounter pin
// the timer1 pin on the 328p is D5
#define HARDWARE_COUNTER_TIMER1 5

// InterruptCounter pin
// 0 = D2, 1 = D3
#define INTERRUPT_COUNTER_PIN 0

// bGeigie <-> xGeigie switch pin
#define GEIGIE_TYPE_PIN A0
#define GEIGIE_TYPE_THRESHOLD 500

// Voltage divider
// GND -- R2 --A7 -- R1 -- VCC
// https://en.wikipedia.org/wiki/Voltage_divider
#define VOLTAGE_PIN A7
#define VOLTAGE_R1 9100
#define VOLTAGE_R2 1000

// SHT1x sensors PIN
#define dataPin  A5
#define clockPin A4

// CO and NOX sensors PIN
#define NOX_Pin  A2
#define CO_Pin   A3

// preheat NOX 
#define PreHeat_Pin   A6

#endif
