/*
   lw_esp_reflow_oven.h

*/

#ifndef _REFLOW_OVEN_H_
#define _REFLOW_OVEN_H_

#include <Arduino.h>

// this is our main .ino filename (saved for showing build info via debug calls)
extern const char *MY_INO_FILE;

#include <stdint.h>
#include <Wire.h>
#include <SPI.h>

#include <TFT_eSPI.h> // Add TFT_eSPI by Bodmer to library


// Install the "XPT2046_Touchscreen" library by Paul Stoffregen to use the Touchscreen - https://github.com/PaulStoffregen/XPT2046_Touchscreen
// Note: this library doesn't require further configuration
#include <XPT2046_Touchscreen.h>


// You also need to set up the X & Y resolution for your touch screen
// (the value it returns for X and Y).  The touch mapping can't
// usually be rotated and is absolute. Change min/max to invert
// planes.

#define TS_MINX 320 //480
#define TS_MINY 0
#define TS_MAXX 0
#define TS_MAXY 240 //320

#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240
#define FONT_SIZE       2

#include "driver/twai.h"      // espressif can-bus api


//
// pins and addrs
//

// Touchscreen pins
#define XPT2046_CLK  25   // T_CLK
#define XPT2046_MOSI 32   // T_DIN
#define XPT2046_CS   33   // T_CS
#define XPT2046_IRQ  36   // T_IRQ
#define XPT2046_MISO 39   // T_OUT




#define VERBOSE_SERIAL_TTY   // turn this OFF for productio

#define EEPROM_SIZE               1500
#define USER_EEPROM_START_ADDR       0 
#define MY_MAGIC_VAL                47
#define SW_VERSION             "v1.0.0"


// canbus constants
#define CANBUS_BAUD_RATE        500  // in 'k' units
#define CANBUS_TX_TICK_COUNT    100 //200
#define CANBUS_RX_TICK_COUNT    100 //200

// user defined message types
#define CANID_REPORT_TEMP       0x0A  //(payload is format 'T0234.75')
#define CANID_SET_RELAY         0x0B  //(payload is single byte: 0=off, 1=on)
#define CANID_REPORT_STATUS     0x0C  //(payload is single byte: 0=safety_shutdown, 1=admin_off, 2=admin_on


// for ESP32-dev CYD
#define _PIN_CAN_RX             22
#define _PIN_CAN_TX             27



//
// protos
//

void canbus_init(uint8_t rx_pin, uint8_t tx_pin);

// 2 versions (char * and byte array)
bool canbus_send(uint16_t msg_id, uint8_t msg_len, uint8_t *msg_bytes);
bool canbus_send(uint16_t msg_id, uint8_t msg_len, char *msg_bytes);

void handle_rx_message(twai_message_t &message);
bool set_remote_ssr(bool ssr_val);

void printState(void);
void drawGrid(void);
void drawButton(int x, int y, int w, int h, uint16_t backgroundColor, uint16_t textColor, String text);

// original version for backward compat
//void writeText(int x, int y, int w, int h, int justification, uint16_t textColor, uint16_t bgTextColor, String text, int8_t xOffset = 0, bool fullLinePadding = false);

// this version takes both x and y offsets for more placement flexability
void writeText(int x, int y, int w, int h, int justification, uint16_t textColor, uint16_t bgTextColor, String text, int8_t xOffset=0, int8_t yOffset=0, bool fullLinePadding = false);

void drawSetupMenu(void);
void drawReflowMenu(void);
void drawEditMenu(String stage, uint16_t bgColor);
int getGridCellX(void);
int getGridCellY(void);
String formatTime(unsigned long milliseconds);
void plotDataPoint(void);
void plotReflowProfile(void);



#endif // _REFLOW_OVEN_H_

// end lw_esp_reflow_oven.h
