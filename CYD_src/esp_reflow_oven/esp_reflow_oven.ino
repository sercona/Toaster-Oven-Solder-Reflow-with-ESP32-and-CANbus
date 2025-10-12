// esp_reflow_oven.ino
//
// updated solder reflow oven for graphical display, using CANbus
//  for remote connectivity of the CYD display and a sensor 'pod'.
//
// original src: https://github.com/aBoyCanDream/Solder-Reflow-Oven
//
// last update: 2025-oct-12 linux-works labs


#include <Arduino.h>

#include "lw_esp_reflow_oven.h"   // project include



// can-bus interface
bool driver_installed = false;


TFT_eSPI tft = TFT_eSPI();       // Invoke custom library


SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);


// add to your library. Search for "PID" by Brett Beauregard
#include <PID_v1.h>

// included with this sketch
#include "my_free_fonts.h" // Include the header file attached to this sketch


#define MAGIC_300 250 //300



const int displayWidth = SCREEN_WIDTH;
const int displayHeight = SCREEN_HEIGHT;
const int gridSize = 53; //80;  // Our 320x240 display is 2/3 the size of the 480x320

// Touchscreen coordinates: (x, y) and pressure (z)
int tp_x, tp_y, tp_z;

bool setupMenu = false;
bool editMenu = false;
bool reflowMenu = false;
//const int touchHoldLimit = 500;

uint32_t touchLastMillis = 0;
uint16_t debounceDelay = 100;

unsigned long timeSinceReflowStarted;
unsigned long timeTempCheck = 1000;
unsigned long lastTimeTempCheck = 0;

double preheatTemp  = 100;
double soakTemp     = 140;
double reflowTemp   = 235;
double cooldownTemp =  25;

unsigned long preheatTime   = 30 * 1000;
unsigned long soakTime      = 90 * 1000;
unsigned long reflowTime    = 70 * 1000;
unsigned long cooldownTime  = 40 * 1000;
unsigned long totalTime = (preheatTime + soakTime + reflowTime + cooldownTime);

bool preheating = false;
bool soaking = false;
bool reflowing = false;
bool coolingDown = false;
bool newState = false;

uint16_t gridColor = 0x7BEF;

uint16_t preheatColor = TFT_RED;
uint16_t soakColor = 0xFBE0;
uint16_t reflowColor = 0xDEE0;
uint16_t cooldownColor = TFT_BLUE; // colors for plotting

uint16_t preheatColor_d = 0xC000;
uint16_t soakColor_d = 0xC2E0;
uint16_t reflowColor_d = 0xC600;
uint16_t cooldownColor_d = 0x0018; // desaturated colors


// Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;



// Specify the links and initial tuning parameters

// Awesome explanation of PID and how to tune the values below if
// you're overshooting or undershooting temp can be found here:
// https://www.youtube.com/watch?v=hRnofMxEf3Q&pp=ygULZGlnaWtleSBwaWQ%3D

double Kp = 2;
double Ki = 5;
double Kd = 1;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);




// protos

void printState(void);
void drawGrid(void);
void drawButton(int x, int y, int w, int h, uint16_t backgroundColor, uint16_t textColor, String text);
void writeText(int x, int y, int w, int h, int justification, uint16_t textColor, uint16_t bgTextColor, String text, int8_t xOffset = 0, bool fullLinePadding = false);
void drawSetupMenu(void);
void drawReflowMenu(void);
void drawEditMenu(String stage, uint16_t bgColor);
int getGridCellX(void);
int getGridCellY(void);
String formatTime(unsigned long milliseconds);
void plotDataPoint(void);
void plotReflowProfile(void);



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
  Serial.print("canbus_send("); Serial.print(msg_id, HEX); Serial.print(", [");
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


bool tp_clicked (void)
{
  if (touchscreen.tirqTouched() && touchscreen.touched()) {
    // Get Touchscreen points
    TS_Point p = touchscreen.getPoint();

    // Calibrate Touchscreen points with map function to the correct width and height

    //tp_x = map(p.x, 200, 3700, 1, SCREEN_WIDTH);
    tp_x = map(p.x, 200, 3700, SCREEN_WIDTH, 1);

    tp_y = map(p.y, 240, 3800, 1, SCREEN_HEIGHT);
    tp_z = p.z;
    return true;
  }

  return false;
}



int GET_X_COORDINATE (void)
{
  if (tp_clicked()) {
    return tp_x;
  } else {
    return 0;
  }
}



int GET_Y_COORDINATE (void)
{
  if (tp_clicked()) {
    return tp_y;
  } else {
    return 0;
  }
}


bool set_remote_ssr (bool ssr_val)
{
  char msg_bytes[8];

  strcpy(msg_bytes, "SSR");     // 'T' packet

  if (ssr_val == true) {
    strcat(msg_bytes, "1");
  } else {
    strcat(msg_bytes, "0");
  }


  // sanity check: make sure we dont exceed 8 bytes
  int l = strlen(msg_bytes);
  if (l > 8) {
#ifdef VERBOSE_SERIAL_TTY
    Serial.println("SSR value cant fit in canbus!");
#endif
    return false;
  }

  canbus_send(CANID_SET_RELAY, l, msg_bytes);

  return true;
}



void handle_rx_message (twai_message_t &message)
{
  char buf[8];
  char send_bytes[8];
  char temp_s[8];

  memset(send_bytes, 0, 8);

#ifdef VERBOSE_SERIAL_TTYx
  Serial.print(millis());  // timestamp
  Serial.print(" can_id: ");
  Serial.println(message.identifier, HEX);
#endif


  // is this a temperature report from the 'pod' ?
  if (message.identifier == CANID_REPORT_TEMP) {   // format "T0026.50"

    // get payload (skip over the initial 'T')
    //memset(buf, 0, 8);
    memcpy(buf, message.data + 1, message.data_length_code - 1);

    // convert from literal ascii to float
    Input = (double)atof(buf);

    Serial.print("remote temperature: ");
    Serial.println(Input);

  } // remote temperature report
}



void setup (void)
{
  delay(1000);

  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }

  Serial.println("\nSolder Reflow Oven");
  delay(100);


  // init canbus
  canbus_init(_PIN_CAN_RX, _PIN_CAN_TX);  // rxpin, txpin
  delay(100);

  // turn off the SSR (safety)
  set_remote_ssr(LOW);



  // Start the SPI for the touchscreen and init the touchscreen
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  // Set the Touchscreen rotation in landscape mode
  // Note: in some displays, the touchscreen might be upside down, so you might need to set the rotation to 3: touchscreen.setRotation(3);
  touchscreen.setRotation(1);


  //tft.begin();  // For original code display
  tft.init();           // Init ST7789 320x240
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.setTextSize(1);


  //PID stuff
  Setpoint = cooldownTemp;

  // tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, 1);

  // turn the PID on
  myPID.SetMode(AUTOMATIC);
}




// Print Touchscreen info about X, Y and Pressure (Z) on the Serial Monitor
void printTouchToSerial(int touchX, int touchY, int touchZ)
{
  Serial.print("X = ");
  Serial.print(touchX);
  Serial.print(" | Y = ");
  Serial.print(touchY);
  Serial.print(" | Pressure = ");
  Serial.print(touchZ);
  Serial.println();
}




// this was a test loop to see if resistive touch works and if the x,y returned are valid.
// only enable this if you need to do testing of the touch screen, etc.  left here just as a docs
// help.
#if 0
void loop1 (void)
{
  // debug touch

  // Checks if Touchscreen was touched, and prints X, Y and Pressure (Z) info on the TFT display and Serial Monitor
  if (touchscreen.tirqTouched() && touchscreen.touched()) {
    // Get Touchscreen points
    TS_Point p = touchscreen.getPoint();
    // Calibrate Touchscreen points with map function to the correct width and height
    tp_x = map(p.x, 200, 3700, 1, SCREEN_WIDTH);
    tp_y = map(p.y, 240, 3800, 1, SCREEN_HEIGHT);
    tp_z = p.z;

    printTouchToSerial(tp_x, tp_y, tp_z);
    //printTouchToDisplay(x, y, z);

    delay(100);
  }
}
#endif



//
// the real loop()
//

void loop (void)
{
  twai_message_t message;
  char send_bytes[8];

  //
  // any CAN messages, for us, on the bus?
  //

  if (twai_receive(&message, pdMS_TO_TICKS(CANBUS_RX_TICK_COUNT)) == ESP_OK) {
    handle_rx_message(message);
  }


  // periodically send ssr updates (starting off at initial state, we ensure its OFF)
  set_remote_ssr(LOW);


  // Setup Menu
  tft.fillScreen(TFT_BLACK);



  drawSetupMenu();
  setupMenu = true;
  Serial.println("Setup Menu");

  while (setupMenu) {

    //
    // any CAN messages, for us, on the bus?
    //

    if (twai_receive(&message, pdMS_TO_TICKS(CANBUS_RX_TICK_COUNT)) == ESP_OK) {
      handle_rx_message(message);
    }

    if (tp_clicked()) {

      if (millis() - touchLastMillis > debounceDelay) {
        Serial.println("touched");

        int setupMenuXPos = getGridCellX();
        int setupMenuYPos = getGridCellY();

        Serial.print("Setup menu touch: (");
        Serial.print(setupMenuXPos);
        Serial.print(",");
        Serial.print(setupMenuYPos);
        Serial.print(") -> ");

        if (setupMenuYPos < 3) { // Somewhere other than the start button
          editMenu = true;
          bool editingPreheat = false, editingSoak = false, editingReflow = false;
          if (setupMenuXPos < 2 ) { // Somwhere within the preheat zone
            editingPreheat = true;
            tft.fillScreen(preheatColor);
            Serial.println("Edit Preheat Menu");

            drawEditMenu("Preheat", preheatColor);
            String preheatTempString = String(int(preheatTemp)) + " C";
            writeText(1, 0, 2, 1, 6, TFT_WHITE, preheatColor, preheatTempString);
            writeText(4, 0, 2, 1, 6, TFT_WHITE, preheatColor, formatTime(preheatTime));
          }

          else if (setupMenuXPos > 3 ) {// Somwhere within the reflow zone
            editingReflow = true;
            tft.fillScreen(reflowColor);
            Serial.println("Edit Reflow Menu");

            drawEditMenu("Reflow", reflowColor);
            String reflowTempString = String(int(reflowTemp)) + " C";
            writeText(1, 0, 2, 1, 6, TFT_WHITE, reflowColor, reflowTempString);
            writeText(4, 0, 2, 1, 6, TFT_WHITE, reflowColor, formatTime(reflowTime));
          }

          else { // Somwhere within the soak zone
            editingSoak = true;
            tft.fillScreen(soakColor);
            Serial.println("Edit Soak Menu");

            drawEditMenu("Soak", soakColor);
            String soakTempString = String(int(soakTemp)) + " C";
            writeText(1, 0, 2, 1, 6, TFT_WHITE, soakColor, soakTempString);
            writeText(4, 0, 2, 1, 6, TFT_WHITE, soakColor, formatTime(soakTime));
          }


          while (editMenu) { // Stay in this loop until the save button is pressed

            //
            // any CAN messages, for us, on the bus?
            //

            if (twai_receive(&message, pdMS_TO_TICKS(CANBUS_RX_TICK_COUNT)) == ESP_OK) {
              handle_rx_message(message);
            }

            if (tp_clicked()) {

              if (millis() - touchLastMillis > debounceDelay) {
                Serial.println("touched");

                int editMenuXPos = getGridCellX();
                int editMenuYPos = getGridCellY();

                Serial.print("Edit menu touch at (");
                Serial.print(editMenuXPos);
                Serial.print(",");
                Serial.print(editMenuYPos);
                Serial.print(") -> ");

                if (editMenuYPos == 1) { // One of the up arrows was pressed
                  if (editMenuXPos < 3) { // The Temp up arrow was pressed
                    Serial.println("Temp up arrow");
                    //tft.fillRoundRect(1*gridSize, 0*gridSize, 2*gridSize, gridSize, 10, TFT_BLACK);

                    if (editingPreheat) {
                      if (preheatTemp < 300) {
                        preheatTemp += 10;
                        String preheatTempString = String(int(preheatTemp)) + " C";
                        writeText(1, 0, 2, 1, 6, TFT_WHITE, preheatColor, preheatTempString);
                      }
                    }

                    if (editingSoak) {
                      if (soakTemp < 300) {
                        soakTemp += 10;
                        String soakTempString = String(int(soakTemp)) + " C";
                        writeText(1, 0, 2, 1, 6, TFT_WHITE, soakColor, soakTempString);
                      }
                    }

                    if (editingReflow) {
                      if (reflowTemp < 300) {
                        reflowTemp += 10;
                        String reflowTempString = String(int(reflowTemp)) + " C";
                        writeText(1, 0, 2, 1, 6, TFT_WHITE, reflowColor, reflowTempString);
                      }
                    }
                  }

                  else { // The Time up arrow was pressed
                    Serial.println("Time up arrow");
                    //tft.fillRoundRect(4*gridSize+2, 0*gridSize+2, 2*gridSize-4, gridSize-4, 10, TFT_BLACK);
                    if (editingPreheat) {
                      if (preheatTime < 300000) {
                        preheatTime += 10000;
                        writeText(4, 0, 2, 1, 6, TFT_WHITE, preheatColor, formatTime(preheatTime));
                      }
                    }

                    if (editingSoak) {
                      if (soakTime < 300000) {
                        soakTime += 10000;
                        writeText(4, 0, 2, 1, 6, TFT_WHITE, soakColor, formatTime(soakTime));
                      }
                    }

                    if (editingReflow) {
                      if (reflowTime < 300000) {
                        reflowTime += 10000;
                        writeText(4, 0, 2, 1, 6, TFT_WHITE, reflowColor, formatTime(reflowTime));
                      }
                    }
                  }
                }

                else if (editMenuYPos == 2) {// One of the down arrows was pressed
                  if (editMenuXPos < 3) { // The Temp down arrow was pressed
                    Serial.println("Temp down arrow");
                    //tft.fillRoundRect(1*gridSize+2, 0*gridSize+2, 2*gridSize-4, gridSize-4, 10, TFT_BLACK);
                    if (editingPreheat) {
                      if (preheatTemp > 100) {
                        preheatTemp -= 10;
                        String preheatTempString = String(int(preheatTemp)) + " C";
                        writeText(1, 0, 2, 1, 6, TFT_WHITE, preheatColor, preheatTempString);
                      }
                    }

                    if (editingSoak) {
                      if (soakTemp > 100) {
                        soakTemp -= 10;
                        String soakTempString = String(int(soakTemp)) + " C";
                        writeText(1, 0, 2, 1, 6, TFT_WHITE, soakColor, soakTempString);
                      }
                    }

                    if (editingReflow) {
                      if (reflowTemp > 100) {
                        reflowTemp -= 10;
                        String reflowTempString = String(int(reflowTemp)) + " C";
                        writeText(1, 0, 2, 1, 6, TFT_WHITE, reflowColor, reflowTempString);
                        //writeText(1,0,2,1, 6, TFT_WHITE, reflowColor, String(int(reflowTemp)));
                      }
                    }
                  }

                  else { // The Time down arrow was pressed
                    Serial.println("Time down arrow");
                    //tft.fillRoundRect(4*gridSize+2, 0*gridSize+2, 2*gridSize-4, gridSize-4, 10, TFT_BLACK);
                    if (editingPreheat) {
                      if (preheatTime > 30000) {
                        preheatTime -= 10000;
                        writeText(4, 0, 2, 1, 6, TFT_WHITE, preheatColor, formatTime(preheatTime));
                      }
                    }

                    else if (editingSoak) {
                      if (soakTime > 30000) {
                        soakTime -= 10000;
                        writeText(4, 0, 2, 1, 6, TFT_WHITE, soakColor, formatTime(soakTime));
                      }
                    }

                    else if (editingReflow) {
                      if (reflowTime > 30000) {
                        reflowTime -= 10000;
                        writeText(4, 0, 2, 1, 6, TFT_WHITE, reflowColor, formatTime(reflowTime));
                      }
                    }
                  }
                }

                else if (editMenuYPos == 3) { // Save button was pressed
                  Serial.println("Save button");
                  tft.fillScreen(TFT_BLACK);
                  drawSetupMenu();
                  editMenu = false;
                }

                //delay(touchHoldLimit); // so holding the button down doesn't read multiple presses
                touchLastMillis = millis();
              }

            }
          }
        }

        else { // Start button was pressed
          Serial.println("Start button");
          setupMenu = false;
        }

        //delay(touchHoldLimit); // so holding the button down doesn't read multiple presses
        touchLastMillis = millis();
      }

    }

  }


  Serial.println("Exiting while setup");

  // Reflow Menu
  tft.fillScreen(TFT_BLACK);

  drawReflowMenu();

  drawButton(0, 3, 2, 1, TFT_GREEN, TFT_WHITE, "Start");

  bool start = false;
  while (!start) {

    //
    // any CAN messages, for us, on the bus?
    //

    if (twai_receive(&message, pdMS_TO_TICKS(CANBUS_RX_TICK_COUNT)) == ESP_OK) {
      handle_rx_message(message);
    }

    if (tp_clicked()) {

      if (millis() - touchLastMillis > debounceDelay) {
        if (getGridCellX() < 2 && getGridCellY() == 3) {
          start = true;
          Serial.println("totalTime: ");
          Serial.println(totalTime);
        }
        //delay(touchHoldLimit); // so holding the button down doesn't read multiple presses
        touchLastMillis = millis();
      }
    }
  }

  drawButton(0, 3, 2, 1, TFT_RED, TFT_WHITE, "Stop");

  Serial.println("Reflow Menu");

  unsigned long reflowStarted = millis();

  reflowMenu = true;
  while (reflowMenu) {
    timeSinceReflowStarted = millis() - reflowStarted;

    //
    // any CAN messages, for us, on the bus?
    //

    if (twai_receive(&message, pdMS_TO_TICKS(CANBUS_RX_TICK_COUNT)) == ESP_OK) {
      handle_rx_message(message);
    }

    if (timeSinceReflowStarted - lastTimeTempCheck > timeTempCheck) {
      lastTimeTempCheck = timeSinceReflowStarted;
      printState();

      Serial.print("\tSetpoint:");
      Serial.print(Setpoint);


      // note, 'Input' is kept updated via a stream of CANbus messages directed to us from the 'pod'


      Serial.print("\tInput:");
      Serial.print(Input);

      myPID.Compute();

      if (Output < 0.5) {
        //digitalWrite(SSR_PIN, LOW);   // do a remote set over canbus
        set_remote_ssr(LOW);
      }

      if (Output > 0.5) {
        //digitalWrite(SSR_PIN, HIGH); // do a remote set over canbus
        set_remote_ssr(HIGH);
      }

      Serial.print("\tOutput:");
      Serial.println(Output);

      plotDataPoint();

    }

    if (timeSinceReflowStarted > totalTime) {
      Serial.println("exit at(timeSinceReflowStarted > totalTime)");
      reflowMenu = false;
    }

    else if (timeSinceReflowStarted > (preheatTime + soakTime + reflowTime)) { // preheat and soak and reflow are complete. Start cooldown
      if (!coolingDown) {
        newState = true;
        preheating = false, soaking = false, reflowing = false, coolingDown = true;
      }
      Setpoint = cooldownTemp;
    }

    else if (timeSinceReflowStarted > (preheatTime + soakTime)) { // preheat and soak are complete. Start reflow
      if (!reflowing) {
        newState = true;
        preheating = false, soaking = false, reflowing = true, coolingDown = false;
      }
      Setpoint = reflowTemp;
    }

    else if (timeSinceReflowStarted > preheatTime) { // preheat is complete. Start soak
      if (!soaking) {
        newState = true;
        preheating = false, soaking = true, reflowing = false, coolingDown = false;
      }
      Setpoint = soakTemp;
    }

    else { // cycle is starting. Start preheat
      if (!preheating) {
        newState = true;
        preheating = true, soaking = false, reflowing = false, coolingDown = false;
      }
      Setpoint = preheatTemp;
    }


    if (tp_clicked()) {

      if (millis() - touchLastMillis > debounceDelay) {

        if (getGridCellX() < 2 && getGridCellY() == 3) {
          Serial.println("exit at stop button");
          reflowMenu = false;
        }

        //delay(touchHoldLimit); // so holding the button down doesn't read multiple presses
        touchLastMillis = millis();
      }
    }
  }

  drawButton(0, 3, 2, 1, TFT_GREEN, TFT_WHITE, "Done");
  bool done = false;
  while (!done) {

    //
    // any CAN messages, for us, on the bus?
    //

    if (twai_receive(&message, pdMS_TO_TICKS(CANBUS_RX_TICK_COUNT)) == ESP_OK) {
      handle_rx_message(message);
    }

    if (tp_clicked()) {

      if (millis() - touchLastMillis > debounceDelay) {
        if (getGridCellX() < 2 && getGridCellY() == 3) {
          done = true;
        }
        //delay(touchHoldLimit); // so holding the button down doesn't read multiple presses
        touchLastMillis = millis();
      }
    }
  }
} // loop




void printState (void)
{
  String time = formatTime(timeSinceReflowStarted);
  Serial.print("Current time: "); Serial.print(time); Serial.print("\t");

  //tft.fillRoundRect(4*gridSize+2, 3*gridSize+2, 2*gridSize-4, gridSize-4, 10, TFT_BLACK);
  writeText(4, 3, 2, 1, 1, TFT_WHITE, TFT_BLACK, time, 10, true);

  String tempReading = String(Input) + " C";
  writeText(4, 3, 2, 1, 3, TFT_WHITE, TFT_BLACK, tempReading, 10, true);

  String currentState;
  if (preheating) {
    currentState = "Preheating";
  }

  if (soaking) {
    currentState = "Soaking";
  }

  if (reflowing) {
    currentState = "Reflowing";
  }

  if (coolingDown) {
    currentState = "Cool Down";
  }

  Serial.print(currentState);

  if (newState) {
    //tft.setFont(&FreeMonoBold12pt7b);
    //tft.setFreeFont(FSB12);
    tft.setFreeFont(FM9);
    newState = false;
    tft.fillRoundRect(1 * gridSize + 25,
                      0 * gridSize + 2,
                      3 * gridSize - 4,
                      gridSize - 20,
                      10,
                      TFT_BLACK);

    writeText(2, 0,
              2, 1,
              4, TFT_WHITE, TFT_BLACK, currentState);
  }
}


void drawGrid (void)
{
  //tft.setFont();
  tft.setTextColor(TFT_WHITE);
  tft.drawRect(0, 0, displayWidth, displayHeight - gridSize, gridColor);

  for (int i = 1; i < 6; i++) {
    tft.drawFastVLine(i * gridSize, 0, displayHeight - gridSize, gridColor);
  }

  for (int j = 1; j < 4; j++) {
    tft.drawFastHLine(0, j * gridSize, displayWidth, gridColor);
  }

  tft.setFreeFont(FM9);

  tft.setTextDatum(TL_DATUM);
  tft.drawString("300", 4, 2);
  //tft.setCursor(4,tft.fontHeight()); tft.print("300");
  tft.drawString("200", 4, 1 * gridSize + 2);
  tft.drawString("100", 4, 2 * gridSize + 2);



  tft.setFreeFont(FSI9);

  tft.setCursor(1 * gridSize + 4,
                3 * gridSize - 7 - 4);
  tft.print(formatTime(totalTime / 6));

  tft.setCursor(2 * gridSize + 4,
                3 * gridSize - 7 - 4);
  tft.print(formatTime(2 * totalTime / 6));

  tft.setCursor(3 * gridSize + 4,
                3 * gridSize - 7 - 4);
  tft.print(formatTime(3 * totalTime / 6));

  tft.setCursor(4 * gridSize + 4,
                3 * gridSize - 7 - 4);
  tft.print(formatTime(4 * totalTime / 6));

  tft.setCursor(5 * gridSize + 4,
                3 * gridSize - 7 - 4);
  tft.print(formatTime(5 * totalTime / 6));

  plotReflowProfile();
}

#define BUTTON_W 46
#define BUTTON_H 40

void drawButton (int x, int y, int w, int h, uint16_t backgroundColor, uint16_t textColor, String text)
{
  //tft.setFont(&FreeMonoBold12pt7b);
  tft.setFreeFont(FM9);
  tft.setTextColor(textColor);

  /*   if (backgroundColor == TFT_BLACK) {
       tft.drawRoundRect(x*gridSize+2, y*gridSize+2, w*gridSize-4, h*gridSize-4, 10, TFT_WHITE);
       }
       else{
       tft.fillRoundRect(x*gridSize+2, y*gridSize+2, w*gridSize-4, h*gridSize-4, 10, backgroundColor);
       } */

  tft.fillRoundRect(x * gridSize + 2,
                    y * gridSize + 2,
                    w * gridSize - 4,
                    h * gridSize - 4,
                    8,
                    backgroundColor);

  if (text == "UP_ARROW") {
    tft.fillTriangle(x * gridSize + (w * gridSize - BUTTON_W) / 2,
                     y * gridSize + (h * gridSize - BUTTON_H) / 2 + BUTTON_H,
                     x * gridSize + (w * gridSize - BUTTON_W) / 2 + BUTTON_W,
                     y * gridSize + (h * gridSize - BUTTON_H) / 2 + BUTTON_H,
                     x * gridSize + w * gridSize / 2,
                     y * gridSize + (h * gridSize - BUTTON_H) / 2,
                     textColor);
  }
  else if (text == "DOWN_ARROW") {
    tft.fillTriangle(x * gridSize + (w * gridSize - BUTTON_W) / 2,
                     y * gridSize + (h * gridSize - BUTTON_H) / 2,
                     x * gridSize + (w * gridSize - BUTTON_W) / 2 + BUTTON_W,
                     y * gridSize + (h * gridSize - BUTTON_H) / 2,
                     x * gridSize + w * gridSize / 2,
                     y * gridSize + (h * gridSize - BUTTON_H) / 2 + BUTTON_H,
                     textColor);
  }
  else {
    tft.setTextDatum(MC_DATUM);

    tft.drawString(text,
                   (x * gridSize) + ((w * gridSize) / 2),
                   (y * gridSize) + ((h * gridSize)) / 2);
  }
}


uint8_t margin = 4;

// justification runs:
// 1 4 7
// 2 5 8
// 3 6 9


void writeText (int x, int y, int w, int h,
                int justification,
                uint16_t textColor, uint16_t bgTextColor,
                String text,
                int8_t xOffset, bool fullLinePadding)
{
  tft.setFreeFont(FSB12);

  int16_t textBoundX, textBoundY;
  uint16_t textBoundWidth, textBoundHeight;

  textBoundWidth = tft.textWidth(text);
  textBoundHeight = tft.fontHeight();

  uint32_t xPos, yPos;

  switch (justification) {
    case 1: //top left
      xPos = x * gridSize + margin + xOffset;
      yPos = y * gridSize + margin;
      tft.setTextDatum(TL_DATUM);
      break;

    case 2: //center left
      xPos = x * gridSize + margin + xOffset;
      yPos = y * gridSize + ((h * gridSize) / 2);
      tft.setTextDatum(CL_DATUM);
      break;

    case 3: //bottom left
      xPos = x * gridSize + margin + xOffset;
      yPos = (y * gridSize) + (h * gridSize) - margin;
      tft.setTextDatum(BL_DATUM);
      break;

    case 4:
      xPos = x * gridSize + ((w * gridSize) / 2) + margin + xOffset;
      yPos = y * gridSize + margin;
      tft.setTextDatum(TC_DATUM);
      break;

    case 5:
      xPos = x * gridSize + ((w * gridSize) / 2) + xOffset;
      yPos = y * gridSize + ((h * gridSize) / 2);
      tft.setTextDatum(CC_DATUM);
      break;

    case 6:
      xPos = x * gridSize + ((w * gridSize) / 2) + margin + xOffset;
      yPos = (y * gridSize) + (h * gridSize) - margin;
      tft.setTextDatum(BC_DATUM);
      break;

    case 7:
      xPos = (x * gridSize) + (w * gridSize) - margin - xOffset;
      yPos = y * gridSize + margin;
      tft.setTextDatum(TR_DATUM);
      break;

    case 8:
      xPos = (x * gridSize) + (w * gridSize) - margin - xOffset;
      yPos = y * gridSize + ((h * gridSize) / 2);
      tft.setTextDatum(CR_DATUM);
      break;

    case 9:
      xPos = (x * gridSize) + (w * gridSize) - margin - xOffset;
      yPos = (y * gridSize) + (h * gridSize) - margin;
      tft.setTextDatum(BR_DATUM);
      break;
  }

  if (fullLinePadding == true) {
    tft.setTextPadding((w * gridSize) - (margin * 2) - xOffset);
  }

  tft.setTextColor(textColor, bgTextColor);

  //
  //
  //tft.setTextDatum(L_BASELINE);
  tft.drawString(text, xPos, yPos);
}



void drawSetupMenu (void)
{
  tft.setFreeFont(FM9);

  drawButton(0, 0, 2, 3, preheatColor, TFT_WHITE, "");
  drawButton(2, 0, 2, 3, soakColor, TFT_WHITE, "");
  drawButton(4, 0, 2, 3, reflowColor, TFT_WHITE, "");

  writeText(0, 0, 2, 1, 5, TFT_WHITE, preheatColor,  "Preheat");
  writeText(2, 0, 2, 1, 5, TFT_WHITE, soakColor, "Soak");
  writeText(4, 0, 2, 1, 5, TFT_WHITE, reflowColor, "Reflow");
  writeText(0, 1, 2, 1, 4, TFT_WHITE, preheatColor, String(int(preheatTemp)) + " C");
  writeText(2, 1, 2, 1, 4, TFT_WHITE, soakColor, String(int(soakTemp)) + " C");
  writeText(4, 1, 2, 1, 4, TFT_WHITE, reflowColor, String(int(reflowTemp)) + " C");
  writeText(0, 1, 2, 1, 6, TFT_WHITE, preheatColor, String(formatTime(preheatTime)));
  writeText(2, 1, 2, 1, 6, TFT_WHITE, soakColor, String(formatTime(soakTime)));
  writeText(4, 1, 2, 1, 6, TFT_WHITE, reflowColor, String(formatTime(reflowTime)));
  writeText(0, 2, 2, 1, 4, TFT_WHITE, preheatColor, "min");
  writeText(2, 2, 2, 1, 4, TFT_WHITE, soakColor, "min");
  writeText(4, 2, 2, 1, 4, TFT_WHITE, reflowColor, "min");

  drawButton(0, 3, 6, 1, TFT_GREEN, TFT_WHITE, "Confirm");

  //tft.drawCircle(95,87,4,TFT_WHITE); tft.drawCircle(255,87,4,TFT_WHITE); tft.drawCircle(415,87,4,TFT_WHITE); // These are the degree circles. They are absolute so need to change to scale. The x axis does not correspond perfectly to 2/3 for some reason
}



void drawReflowMenu (void)
{
  tft.setFreeFont(FM9);

  drawGrid();

  writeText(3, 3, 1, 1, 7, TFT_WHITE, TFT_BLACK, "Time: ");
  writeText(3, 3, 1, 1, 9, TFT_WHITE, TFT_BLACK, "Temp: ");

  //drawButton(0,3,2,1, TFT_RED, TFT_WHITE, "Stop");
  drawButton(0, 3, 2, 1, TFT_RED, TFT_WHITE, "Start");
}



void drawEditMenu (String stage, uint16_t bgColor)
{
  tft.setFreeFont(FM9);

  // what stage we are editing
  writeText(0, 0, 6, 1, 4, TFT_WHITE, bgColor, stage);


  // temp
  writeText(0, 0, 2, 1, 3, TFT_CYAN, bgColor, "Temp:", 10);

  drawButton(0, 1, 3, 1, TFT_WHITE, TFT_BLACK, "UP_ARROW");
  drawButton(0, 2, 3, 1, TFT_WHITE, TFT_BLACK, "DOWN_ARROW");


  // time
  writeText(3, 0, 2, 1, 3, TFT_CYAN, bgColor, "Time:", 10);

  drawButton(3, 1, 3, 1, TFT_WHITE, TFT_BLACK, "UP_ARROW");
  drawButton(3, 2, 3, 1, TFT_WHITE, TFT_BLACK, "DOWN_ARROW");

  drawButton(0, 3, 6, 1, TFT_GREEN, TFT_WHITE, "Save");
}



int getGridCellX (void)
{
  //int xpoint = touchpoint.x;
  int xpoint = GET_X_COORDINATE();

  Serial.print("x resistance: "); Serial.print(xpoint); Serial.print(" ");

  xpoint = map(xpoint, TS_MINX, TS_MAXX, displayWidth - 1, 0);

  if (xpoint < gridSize)
    return 5;

  else if (xpoint < gridSize * 2)
    return 4;

  else if (xpoint < gridSize * 3)
    return 3;

  else if (xpoint < gridSize * 4)
    return 2;

  else if (xpoint < gridSize * 5)
    return 1;

  else
    return 0;
}


int getGridCellY (void)
{
  int ypoint = GET_Y_COORDINATE();

  Serial.print("y resistance: "); Serial.print(ypoint); Serial.print(" ");

  ypoint = map(ypoint, TS_MINY, TS_MAXY, 0, displayHeight - 1);
  if (ypoint < gridSize)
    return 0;

  else if (ypoint < gridSize * 2)
    return 1;

  else if (ypoint < gridSize * 3)
    return 2;

  else
    return 3;
}


String formatTime (unsigned long milliseconds)
{
  // Calculate the number of minutes and seconds
  unsigned long totalSeconds = milliseconds / 1000;
  unsigned int minutes = totalSeconds / 60;
  unsigned int seconds = totalSeconds % 60;

  // Format the time as a string with a leading zero if necessary
  //String formattedTime = (minutes < 10 ? "0" : "") + String(minutes) + ":" + (seconds < 10 ? "0" : "") + String(seconds);

  // format it as simple seconds
  String formattedTime = String(totalSeconds);

  return formattedTime;
}



void plotDataPoint (void)
{
  uint16_t color;

  if (preheating) {
    color = preheatColor;
  }

  if (soaking) {
    color = soakColor;
  }

  if (reflowing) {
    color = reflowColor;
  }

  if (coolingDown) {
    color = cooldownColor;
  }

  tft.fillCircle(map(timeSinceReflowStarted, 0, totalTime, 0, displayWidth),  // x (time)
                 // now the y-value (needs some fixing)
                 map(Input,
                     0,
                     MAGIC_300 + 50,
                     3 * gridSize,
                     0),
                 2,
                 color);

  //tft.fillCircle(mapTime(timeSinceReflowStarted), mapTemp(Input), 2, color);
}


void plotReflowProfile (void)
{
  int xMin, xMax, amp;

  xMin = 0;
  xMax = map(preheatTime, 0, totalTime, 0, displayWidth);
  amp = map(preheatTemp, 0, (MAGIC_300 + 50), 3 * gridSize, 0) - map(cooldownTemp, 0, (MAGIC_300 + 50), 3 * gridSize, 0);

  for (int i = 0; i <= (xMax - xMin); i++) {
    tft.fillCircle(xMin + i, -amp / 2 * cos(PI * i / (xMax - xMin)) + map(cooldownTemp, 0, 300, 3 * gridSize, 0) + amp / 2, 2, preheatColor_d);
  }

  xMin = map(preheatTime, 0, totalTime, 0, displayWidth);
  xMax = map(preheatTime + soakTime, 0, totalTime, 0, displayWidth);
  amp = map(soakTemp, 0, (MAGIC_300 + 50), 3 * gridSize, 0) - map(preheatTemp, 0, (MAGIC_300 + 50), 3 * gridSize, 0);

  //amp = 80;
  for (int i = 0; i <= (xMax - xMin); i++) {
    tft.fillCircle(xMin + i, -amp / 2 * cos(PI * i / (xMax - xMin)) + map(preheatTemp, 0, (MAGIC_300 + 50), 3 * gridSize, 0) + amp / 2, 2, soakColor_d);
  }

  xMin = map(preheatTime + soakTime, 0, totalTime, 0, displayWidth);
  xMax = map(preheatTime + soakTime + reflowTime, 0, totalTime, 0, displayWidth);
  amp = map(reflowTemp, 0, (MAGIC_300 + 50), 3 * gridSize, 0) - map(soakTemp, 0, (MAGIC_300 + 50), 3 * gridSize, 0);

  //amp = 80;
  for (int i = 0; i <= (xMax - xMin); i++) {
    tft.fillCircle(xMin + i, -amp / 2 * cos(PI * i / (xMax - xMin)) + map(soakTemp, 0, (MAGIC_300 + 50), 3 * gridSize, 0) + amp / 2, 2, reflowColor_d);
  }

  xMin = map(preheatTime + soakTime + reflowTime, 0, totalTime, 0, displayWidth);
  xMax = map(totalTime, 0, totalTime, 0, displayWidth);
  amp = map(cooldownTemp, 0, (MAGIC_300 + 50), 3 * gridSize, 0) - map(reflowTemp, 0, (MAGIC_300 + 50), 3 * gridSize, 0);

  //amp = 80;
  for (int i = 0; i <= (xMax - xMin); i++) {
    tft.fillCircle(xMin + i, -amp / 2 * cos(PI * i / (xMax - xMin)) + map(reflowTemp, 0, (MAGIC_300 + 50), 3 * gridSize, 0) + amp / 2, 2, cooldownColor_d);
  }
}

// end esp_reflow_oven.ino
