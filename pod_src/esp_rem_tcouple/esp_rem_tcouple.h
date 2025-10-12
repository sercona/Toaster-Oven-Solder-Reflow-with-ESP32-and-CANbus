/*
  esp_rem_tcouple.h
*/

#ifndef _TCOUPLE_
#define _TCOUPLE_

#include <Arduino.h>

// this is our main .ino filename (saved for showing build info via debug calls)
extern const char *MY_INO_FILE;

#include "driver/twai.h"      // espressif can-bus api

#include <SPI.h>


#define VERBOSE_SERIAL_TTY   // turn this OFF for productio

#define EEPROM_SIZE               1500
#define USER_EEPROM_START_ADDR       0  // for the demo, pick something other than the base (0)
#define MY_MAGIC_VAL                46
#define SW_VERSION             "v1.0.0"


// canbus constants
#define CANBUS_BAUD_RATE        500  // in 'k' units
#define CANBUS_TX_TICK_COUNT    200 //200
#define CANBUS_RX_TICK_COUNT    200 //200

// user defined message types
#define CANID_REPORT_TEMP       0x0A  //(payload is format 'T0234.75')
#define CANID_SET_RELAY         0x0B  //(payload is single byte: 0=off, 1=on)
#define CANID_REPORT_STATUS     0x0C  //(payload is single byte: 0=safety_shutdown, 1=admin_off, 2=admin_on


#include <max6675.h>

#ifdef ARDUINO_ESP32C3_DEV
#define _PIN_thermoDO           4
#define _PIN_thermoCS           1
#define _PIN_thermoCLK          5

// for TINY-can board and c3 chips
#define _PIN_CAN_RX             6
#define _PIN_CAN_TX             7

#define _PIN_SSR                0
#endif


// enums (bool) for ssr/solid state relay
// if the led is inverted, this is what you write to the port to get it to do what you want
#define SSR_STAT_OFF  0
#define SSR_STAT_ON   1


// SSR status
extern uint32_t last_canbus_msg_ts; // = 0;
extern bool ssr_status; // = SSR_STAT_OFF;   // 0=off, 1=on
extern bool ssr_state_ok; // = false;



// protos
void canbus_init(uint8_t rx_pin, uint8_t tx_pin);

// 2 versions (char * and byte array)
bool canbus_send(uint16_t msg_id, uint8_t msg_len, uint8_t *msg_bytes);
bool canbus_send(uint16_t msg_id, uint8_t msg_len, char *msg_bytes);

void handle_rx_message(twai_message_t &message);
bool send_temperature(int t_val);
double double_map(double x, double in_min, double in_max, double out_min, double out_max);
void update_ssr(uint8_t val);



#endif // _TCOUPLE_
