/*
  esp_rem_tcouple


  on linux, if you use can-utils, you can setup aliases to turn the ssr and off, to test:

  $ alias ron  'cansend can0 00B#53535231'
  $ alias roff 'cansend can0 00B#53535230'


  (c) 2025 linux-works labs
*/

#include <Arduino.h>

const char *MY_INO_FILE = __FILE__;  // capture the .ino filename and save it

#include "esp_rem_tcouple.h"


// can-bus interface
bool driver_installed = false;

// tcouple driver
MAX6675 thermocouple(_PIN_thermoCLK, _PIN_thermoCS, _PIN_thermoDO);

// SSR status
uint32_t last_canbus_msg_ts = millis();
bool ssr_status = SSR_STAT_OFF;   // 0=off, 1=on
bool ssr_state_ok = false;



double double_map (double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



void canbus_init (uint8_t rx_pin, uint8_t tx_pin)
{
#ifdef VERBOSE_SERIAL_TTY
  Serial.print("canbus pins: tx="); Serial.println(tx_pin);
  Serial.print("canbus pins: rx="); Serial.println(rx_pin);
#endif


  // Initialize configuration structures using macro initializers
  twai_timing_config_t  t_config;
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)tx_pin, (gpio_num_t)rx_pin, TWAI_MODE_NORMAL);
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (CANBUS_BAUD_RATE == 500) {
    t_config = TWAI_TIMING_CONFIG_500KBITS();
  } else {
    t_config = TWAI_TIMING_CONFIG_1MBITS();
  }

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
#ifdef VERBOSE_SERIAL_TTY
    Serial.println("canbus Driver installed");
#endif

  } else {
#ifdef VERBOSE_SERIAL_TTY
    Serial.println("canbus Failed to install driver");
#endif

    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
#ifdef VERBOSE_SERIAL_TTY
    Serial.println("canbus Driver started");
#endif

  } else {
#ifdef VERBOSE_SERIAL_TTY
    Serial.println("canbus Failed to start driver");
#endif

    return;
  }


  // Reconfigure alerts to detect TX alerts and Bus-Off errors
  uint32_t alerts_to_enable = /*TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | */  TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_ERROR;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
#ifdef VERBOSE_SERIAL_TTY
    Serial.println("TWAI Alerts: reconfigured ok");
#endif

  } else {
#ifdef VERBOSE_SERIAL_TTY
    Serial.println("TWAI Alerts: Failed to reconfigure");
#endif

    return;
  }

  // TWAI driver is now successfully installed and started
  driver_installed = true;
}


// char * version of Send message
bool canbus_send (uint16_t msg_id, uint8_t msg_len, char *msg_bytes)
{
  return canbus_send(msg_id, msg_len, (uint8_t *)msg_bytes);
}



// byte array version of Send message
bool canbus_send (uint16_t msg_id, uint8_t msg_len, uint8_t *msg_bytes)
{
#ifdef VERBOSE_SERIAL_TTYx
  Serial.print("_canbus_send("); Serial.print(msg_id, HEX); Serial.print(", [");
  for (int i = 0; i < msg_len; i++) {
    Serial.print((char)msg_bytes[i]);
  }
  Serial.println("])");
#endif

  // alloc message struct
  twai_message_t message;

  message.identifier = msg_id;

  // these are always 0
  message.extd = 0;
  message.rtr = 0;

  message.ss = 0;  // not a single shot (let it retry by itself)
  message.self = 0;
  message.dlc_non_comp = 0;

  // up to 8 bytes of payload
  message.data_length_code = msg_len;
  memcpy(message.data, msg_bytes, msg_len);


  // Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(CANBUS_TX_TICK_COUNT)) == ESP_OK) {
    return 1;  // success

  } else {
    return 0;  // fail
  }
}



bool send_temperature (float t_val)
{
  char msg_bytes[8];
  char temp_s[8];

  memset(msg_bytes, 0, 8);


  // convert to literal ascii
  dtostrf((double)t_val, 7, 2, temp_s);

  // replace spaces with '0'
  for (int i = 0; i < 8; i++) {
    if (temp_s[i] == ' ') {
      temp_s[i] = '0';
    }
  }

  // make the message
  strcpy(msg_bytes, "T");     // 'T' packet
  strcat(msg_bytes, temp_s);  // add temp value


  // sanity check: make sure we dont exceed 8 bytes
  int l = strlen(msg_bytes);
  if (l > 8) {
#ifdef VERBOSE_SERIAL_TTY
    Serial.println("temp value cant fit in canbus!");
#endif
    return false;
  }

  canbus_send(CANID_REPORT_TEMP, l, msg_bytes);

  return true;
}


void handle_rx_message (twai_message_t &message)
{
  uint8_t buf[8];
  char send_bytes[8];

  memset(send_bytes, 0, 8);

#ifdef VERBOSE_SERIAL_TTYx
  Serial.print(millis());  // timestamp
  Serial.print(" can_id: ");
  Serial.println(message.identifier, HEX);
#endif


  if (message.identifier == CANID_SET_RELAY) {

    // get payload
    memset(buf, 0, 8);
    memcpy(buf, message.data, message.data_length_code);
    //for (int i = 0; i < message.data_length_code; i++) {
    //  buf[i] = message.data[i];
    //}

    Serial.print("Relay State: ");
    Serial.println((char)buf[3]);


    // "SSR0" or "SSR1"
    // parse the packet and change ssr status if valid

    if (buf[3] == '0') {
      last_canbus_msg_ts = millis();  // update our timestamp
      ssr_status = SSR_STAT_OFF;   // 0=off, 1=on
      update_ssr(ssr_status);
      if (ssr_state_ok == false) {
        Serial.println("SSR heartbeat resumed.\n");
        ssr_state_ok = true;
        strcpy(send_bytes, "STAT:");
        strcat(send_bytes, "1");
        canbus_send(CANID_REPORT_STATUS, strlen(send_bytes), send_bytes);   //(payload is single byte: 0=safety_shutdown, 1=admin_off, 2=admin_on
      }
      Serial.println("SSR state: ADMIN-OFF");


    } else if (buf[3] == '1') {
      last_canbus_msg_ts = millis();  // update our timestamp
      ssr_status = SSR_STAT_ON;   // 0=off, 1=on
      update_ssr(ssr_status);
      if (ssr_state_ok == false) {
        Serial.println("SSR heartbeat resumed.\n");
        ssr_state_ok = true;
        strcpy(send_bytes, "STAT:");
        strcat(send_bytes, "2");
        canbus_send(CANID_REPORT_STATUS, strlen(send_bytes), send_bytes);   //(payload is single byte: 0=safety_shutdown, 1=admin_off, 2=admin_on
      }
      Serial.println("SSR state: ADMIN-ON");
    }

  } // remote relay set-request
}


// in: 0=off, 1=on
void update_ssr (uint8_t val)
{
  if (val == 0) {
    ssr_status = SSR_STAT_OFF;   // 0=off, 1=on
    digitalWrite(_PIN_SSR, ssr_status);

  } else if (val == 1) {
    ssr_status = SSR_STAT_ON;   // 0=off, 1=on
    digitalWrite(_PIN_SSR, ssr_status);
  }
}



void setup (void)
{
  char send_bytes[8];

  
  delay(1000);

  Serial.begin(115200);
  while (!Serial) {
    delay(100);
    yield();
  }
  Serial.println("\nMAX6675 test");

  // wait for MAX chip to stabilize
  delay(300);

  // SSR output pin
  pinMode(_PIN_SSR, OUTPUT);
  ssr_status = SSR_STAT_OFF;   // 0=off, 1=on
  ssr_state_ok = false;
  update_ssr(ssr_status);


  // init canbus
  canbus_init(_PIN_CAN_RX, _PIN_CAN_TX);  // rxpin, txpin

  // start off telling remote station we are in 'safe' mode
  strcpy(send_bytes, "STAT:");
  strcat(send_bytes, "0");
  canbus_send(CANID_REPORT_STATUS, strlen(send_bytes), send_bytes);

  // debug only!
  //ssr_status = SSR_STAT_ON;   // 0=off, 1=on
  //update_ssr(ssr_status);

  last_canbus_msg_ts = millis();  // reset timer
}


void loop()
{
  twai_message_t message;
  char send_bytes[8];
  float temp_c;


  //
  // any CAN messages, for us, on the bus?
  //

  if (twai_receive(&message, pdMS_TO_TICKS(CANBUS_RX_TICK_COUNT)) == ESP_OK) {
    handle_rx_message(message);
  }


  //
  // if we have not gotton an SSR command recently enough, turn off the ssr and wait
  //  until we hear back again

  if ( (millis() - last_canbus_msg_ts) > (3 * 1000) ) { // is 3 seconds enough?

    if (ssr_state_ok == true) {
      Serial.println("warning, no SSR command for 3 seconds, turning SSR off.");
      ssr_state_ok = false;
      strcpy(send_bytes, "STAT:");
      strcat(send_bytes, "0");
      canbus_send(CANID_REPORT_STATUS, strlen(send_bytes), send_bytes);   //(payload is single byte: 0=safety_shutdown, 1=admin_off, 2=admin_on
    }

    ssr_status = SSR_STAT_OFF;  // emergency off
    update_ssr(ssr_status);
    // note, dont update the timestamp until we get a valid SET from the remote end (again)
  }



  // show SSR current status
  if (ssr_status == SSR_STAT_ON) {
    Serial.println(" SSR is ADMIN-ON");

  } else if (ssr_status == SSR_STAT_OFF) {
    if (ssr_state_ok == true) {
      Serial.println(" SSR is ADMIN-OFF");
    } else {
      Serial.println(" SSR is SAFETY-OFF");
    }

  } else {
    Serial.println(" SSR status is NOT VALID!");
  }


  //
  // read the thermocouple
  //

  temp_c = thermocouple.readCelsius();

  if (temp_c == NAN) {
    Serial.println("Error in tcouple!");
    delay(500);
    return;
  }

  Serial.print("Temp degrees C = "); Serial.println(temp_c); Serial.println();


  // send updates over canbus
  send_temperature(temp_c);


  // For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
  delay(300);
}
