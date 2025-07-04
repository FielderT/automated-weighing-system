#include <Arduino.h>
#include <HX711_ADC.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiManager.h> 
#include <HTTPClient.h>

// --- Pin Definitions ---
static constexpr int HX711_DOUT     = 18;
static constexpr int HX711_SCK      = 19;
static constexpr int CAPTURE_BTN    = 12;
static constexpr int TARE_BTN       = 13;
static constexpr int CALIBRATE_BTN  = 2;
static constexpr int RESET_BTN      = 15;
static constexpr int BATTERY_PIN    = 35;
static constexpr int LCD_POWER_PIN  = 17;
static constexpr int USB_SENSE_PIN  = 34;

static constexpr float VOLT_DIV     = 2.15f;
static constexpr float ADC_REF      = 3.3f;
static constexpr int   ADC_RES      = 4095;
static constexpr float USB_THRESH   = 1.0f;

static constexpr int   KNOWN_MASS   = 1000;    // grams for calibration
static constexpr uint32_t IDLE_TOUT = 120000;  // ms until deep-sleep

static constexpr float presence_threshold      = 0.200f;  // kg
static constexpr int   stable_window           = 50;
static constexpr unsigned long stability_interval = 50;    // ms
static constexpr float stability_max_deviation = 0.010f;   // kg

static constexpr uint32_t DEBOUNCE_MS = 50;
static constexpr unsigned long printInterval = 500;  // battery update
static constexpr unsigned long LIVE_INTERVAL = 200;  // live weight update

// Ganti dengan IP PC yang menjalankan XAMPP
const char* serverName = "http://192.168.0.100/insert_weight.php";  

enum State { STATE_IDLE, STATE_CHICKEN_ON };
enum ManualState { MANUAL_IDLE, MANUAL_MEASURING };

LiquidCrystal_I2C lcd(0x27, 20, 4);
HX711_ADC loadCell(HX711_DOUT, HX711_SCK);

float calibrationValue = 0.0f;
static const int EEPROM_ADDR_CAL = 0;

// Rolling buffer for stability check
float weightBuffer[stable_window];
int   bufferIndex   = 0;
bool  bufferFilled  = false;

// Debounce timers
uint32_t lastCaptureBtnTime = 0;
uint32_t lastTareBtnTime    = 0;
uint32_t lastCalBtnTime     = 0;

bool batteryNeedUpdate = false;  // Flag to update battery display

// Mode & state
bool autoCaptureEnabled = false;  // false = manual
State currentState      = STATE_IDLE;

// Activity & display
uint32_t lastActivityTime = 0;
unsigned long lastPrintTime = 0;

// Live weight display
unsigned long lastLiveTs   = 0;
float         lastRawDisplay = -1.0f;

// Capture display
bool  capturingDisplayed   = false;
float lastCapturedWeight   = 0.0f;

byte batteryIcon[8] = {
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

byte chargingIcon[8] = {
  0b00011,
  0b00110,
  0b01100,
  0b11111,
  0b00011,
  0b00110,
  0b01100,
  0b11000
};

bool isWifiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

// ----------------------------------------------------------------------------
// Deep-sleep
// ----------------------------------------------------------------------------
void prepareDeepSleep() {
  lcd.clear(); lcd.setCursor(0,0);  lcd.print("Sleeping...");
  lcd.setCursor(0,1); lcd.print("Press button");
  lcd.noBacklight();
  delay(200);
  esp_deep_sleep_start();
}

// ----------------------------------------------------------------------------
// showLiveWeight(): update only when two-decimal value changes
// ----------------------------------------------------------------------------
void showLiveWeight() {
  unsigned long now = millis();
  if (now - lastLiveTs < LIVE_INTERVAL) return;
  lastLiveTs = now;

  loadCell.update();  // ensure we have fresh data

  float raw = loadCell.getData() / 1000.0f;  // kg
  float rounded = roundf(raw * 100.0f) / 100.0f;  // two decimals

  if (fabs(rounded - lastRawDisplay) < 0.005f) return;
  lastRawDisplay = rounded;

  lcd.setCursor(8, 1);
  lcd.print("       ");
  lcd.setCursor(8, 1);
  lcd.print(rounded, 2);
  lcd.print("kg");
}

void sendWeightDB(float weight){
  HTTPClient http;

  String serverPath = String(serverName) + "?weight=" + weight;
  Serial.println("Mengirim ke server: " + serverPath);
  http.begin(serverPath);
  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("Response dari server: " + response);
  } else {
    Serial.printf("Error saat mengirim data: %s\n", http.errorToString(httpResponseCode).c_str());
  }
  http.end();

}

// ----------------------------------------------------------------------------
// autoCaptureWeight(): returns >0 on capture, 0 on off, -1 no event
// ----------------------------------------------------------------------------
float autoCaptureWeight() {
  static unsigned long lastSampleTs = 0;
  if (!loadCell.update() || millis() - lastSampleTs < stability_interval)
    return -1.0f;
  lastSampleTs = millis();

  float raw = loadCell.getData() / 1000.0f;
  weightBuffer[bufferIndex++] = raw;
  if (bufferIndex >= stable_window) {
    bufferIndex  = 0;
    bufferFilled = true;
  }
  if (!bufferFilled) return -1.0f;

  float mn = weightBuffer[0], mx = weightBuffer[0], sum = 0;
  for (int i=0; i<stable_window; i++) {
    float v = weightBuffer[i];
    sum += v;
    mn  = min(mn, v);
    mx  = max(mx, v);
  }
  float avg = sum / stable_window;
  float dev = mx - mn;

  auto transition = [&](State s, float ret) {
    currentState  = s;
    bufferFilled  = false;
    bufferIndex   = 0;
    return ret;
  };

  if (currentState == STATE_IDLE) {
    if (avg > presence_threshold && dev < stability_max_deviation) {
      lastCapturedWeight = roundf(avg * 100.0f) / 100.0f;
      capturingDisplayed = false;

      if(isWifiConnected()) {
        // Send captured weight to server or handle it as needed
        lcd.setCursor(0,2);
        lcd.print("Captured: " + String(lastCapturedWeight, 2) + " kg");
        sendWeightDB(lastCapturedWeight);
      } else{
        lcd.setCursor(0,2);
        lcd.print("Wi-Fi not connected");
      }
      return transition(STATE_CHICKEN_ON, lastCapturedWeight);
    }
  } else {
    if (avg < presence_threshold && dev < stability_max_deviation) {
      lcd.setCursor(0,2);
      lcd.print("                    "); // Clear line
      return transition(STATE_IDLE, 0.0f);
    }
  }
  return -1.0f;
}

// ----------------------------------------------------------------------------
// manualCaptureWeight(): returns >0 on capture, -1 otherwise
// ----------------------------------------------------------------------------
float manualCaptureWeight() {
  static ManualState  mState     = MANUAL_IDLE;
  static unsigned long lastSample = 0;

  // Idle: wait for button press
  if (mState == MANUAL_IDLE) {
    if (digitalRead(CAPTURE_BTN) == LOW
        && millis() - lastCaptureBtnTime > DEBOUNCE_MS) {
      lastCaptureBtnTime = millis();
      bufferFilled      = false;
      bufferIndex       = 0;
      lastSample        = 0;
      mState            = MANUAL_MEASURING;
      capturingDisplayed = false;
      lcd.setCursor(0,2);
      lcd.print("Measuring...");
    }
    return -1.0f;
  }

  // Measuring: sample at interval
  if (!loadCell.update() || millis() - lastSample < stability_interval)
    return -1.0f;
  lastSample = millis();

  float raw = loadCell.getData() / 1000.0f;
  weightBuffer[bufferIndex++] = raw;
  if (bufferIndex >= stable_window) {
    bufferIndex  = 0;
    bufferFilled = true;
  }
  if (!bufferFilled) return -1.0f;

  float mn = weightBuffer[0], mx = weightBuffer[0], sum = 0;
  for (int i=0; i<stable_window; i++) {
    float v = weightBuffer[i];
    sum += v;
    mn  = min(mn, v);
    mx  = max(mx, v);
  }
  float avg = sum / stable_window;
  float dev = mx - mn;

  if (avg > presence_threshold && dev < stability_max_deviation) {
    lastCapturedWeight = roundf(avg * 100.0f) / 100.0f;
    mState            = MANUAL_IDLE;
    
    if(isWifiConnected()) {
      // Send captured weight to server or handle it as needed
      lcd.setCursor(0,2);
      lcd.print("Captured: " + String(lastCapturedWeight, 2) + " kg");
      sendWeightDB(lastCapturedWeight);
    } else{
      lcd.setCursor(0,2);
      lcd.print("Wi-Fi not connected");
    }
    return lastCapturedWeight;
  } 
  return -1.0f;
}

// ----------------------------------------------------------------------------
// Stub: update battery display (unchanged from yours)
// ----------------------------------------------------------------------------
// Simplified readBattery() with “only redraw when changed”
void readBattery() {
  static int prevLevel = -1;
  int raw = analogRead(BATTERY_PIN);
  float voltage = (raw / (float)ADC_RES) * ADC_REF * VOLT_DIV;
  bool usb = (analogRead(USB_SENSE_PIN) * 3.3f / 4095.0f) > USB_THRESH;
  if (usb) {
    voltage -= 0.08f;
    lcd.setCursor(13,0);
    lcd.write(byte(1));
  } else {
    lcd.setCursor(13,0);
    lcd.print(" ");
  }

  int lvl;
  if (voltage >= 4.00f)       lvl = 3;
  else if (voltage >= 3.85f)  lvl = 2;
  else if (voltage >= 3.7f)  lvl = 1;
  else if (voltage > 3.6f)  lvl = 0;

  if (lvl != prevLevel || batteryNeedUpdate) {
    prevLevel = lvl;
    lcd.setCursor(14,0);
    lcd.print("      ");
    lcd.setCursor(14,0);
    lcd.print("[");
    for (int i = 0; i <= lvl; i++) lcd.write(byte(0)); // print bar sesuai level
    lcd.setCursor(19,0);
    lcd.print("]");
    batteryNeedUpdate = false;  // reset flag
  }

  if (voltage <= 3.55f && !usb) {
    Serial.println("Battery Low!");
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("Battery Low...");
    lcd.setCursor(0,2);
    lcd.print("Please Charge...");
    // startBeep();
    delay(5000);
    prepareDeepSleep();
  }
}


// ----------------------------------------------------------------------------
// Calibration & Tare handlers (with LCD feedback)
// ----------------------------------------------------------------------------
// 3‐step calibration (no commented code)
void calibrate() {
  uint32_t startTime = millis();
  const uint32_t TIMEOUT = 60000;
  // 1) Tare phase
  lcd.setCursor(0,1); lcd.print("Taring Scale...    ");
  lcd.setCursor(0,2); lcd.print("Remove weight      ");
  lcd.setCursor(0,3); lcd.print("Press button       ");
  while (millis() - startTime < TIMEOUT) {
    loadCell.update();
    if (digitalRead(CAPTURE_BTN) == LOW) {
      delay(50); while (digitalRead(CAPTURE_BTN) == LOW);
      loadCell.tareNoDelay();
    }
    if (loadCell.getTareStatus()) {
      lcd.setCursor(0,1); lcd.print("Tare Done          ");
      lcd.setCursor(0,2); lcd.print("                   ");
      lcd.setCursor(0,3); lcd.print("                   ");
      delay(1000);
      break;
    }
  }
  if (millis() - startTime >= TIMEOUT) return; // timed out

  // 2) Place known mass & confirm
  lcd.setCursor(0,1); lcd.print("Place 1kg weight   ");
  lcd.setCursor(0,2); lcd.print("Press button       ");
  lcd.setCursor(0,3); lcd.print("                   ");
  startTime = millis();
  while (millis() - startTime < TIMEOUT) {
    loadCell.update();
    if (digitalRead(CAPTURE_BTN) == LOW) {
      while (digitalRead(CAPTURE_BTN) == LOW);
      break;
    }
  }
  if (millis() - startTime >= TIMEOUT) return; // timed out

  // 3) Compute & store
  loadCell.refreshDataSet();
  float newCal = loadCell.getNewCalibration(KNOWN_MASS);
  if (abs(newCal - calibrationValue) > 0.01f) {
    calibrationValue = newCal;
    EEPROM.put(EEPROM_ADDR_CAL, calibrationValue);
    EEPROM.commit();
  }
  loadCell.setCalFactor(calibrationValue);
  lcd.setCursor(0,1);
  lcd.print("Calibration Done   ");
  lcd.setCursor(0,2); lcd.print("                   ");
  lcd.setCursor(0,3); lcd.print("                   ");
  delay(1000);
}

void handleCalibration() {
  calibrate();
  // ==== redraw main menu ====
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("NT Scale");
  lcd.setCursor(0,1);
  lcd.print("Weight: ");
  lcd.setCursor(0,3);
  lcd.print(autoCaptureEnabled ? "Mode: Auto" : "Mode: Manual");
  capturingDisplayed = false;
  currentState = STATE_CHICKEN_ON; // reset state
  batteryNeedUpdate = true; // force battery update
  lastActivityTime = millis();
}

void handleTare() {
  lcd.clear(); lcd.setCursor(0,0); lcd.print("Taring...");
  delay(500);
  loadCell.tare();

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("NT Scale");
  lcd.setCursor(0,1);
  lcd.print("Weight: ");
  lcd.setCursor(8,1);
  lcd.print("0.00kg");
  lcd.setCursor(0,3);
  lcd.print(autoCaptureEnabled ? "Mode: Auto" : "Mode: Manual");
  capturingDisplayed = false;
  currentState = STATE_IDLE; // reset state
  batteryNeedUpdate = true; // force battery update
  lastActivityTime = millis();
}






// ----------------------------------------------------------------------------
// Setup
// ----------------------------------------------------------------------------
void setup(){
  Serial.begin(115200);
  pinMode(BATTERY_PIN, INPUT);
  pinMode(USB_SENSE_PIN, INPUT);

  pinMode(CAPTURE_BTN,   INPUT_PULLUP);
  pinMode(TARE_BTN,      INPUT_PULLUP);
  pinMode(CALIBRATE_BTN, INPUT_PULLUP);
  pinMode(RESET_BTN,     INPUT_PULLUP);
  pinMode(LCD_POWER_PIN, OUTPUT);

  // LCD init
  digitalWrite(LCD_POWER_PIN, HIGH);
  lcd.init(); lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Initializing...");

  // Create WiFiManager object
  WiFiManager wm;

  if (digitalRead(RESET_BTN) == LOW) {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("NT Scale");
    lcd.setCursor(0,1); lcd.print("SSID: ESP32-Setup");
    lcd.setCursor(0,2); lcd.print("Pass: 12345678");
    lcd.setCursor(0,3); lcd.print("Browser: 192.168.4.1");
    wm.resetSettings();  // Reset WiFi settings
    Serial.println("WiFi settings reset. Restarting...");
  }

  // Try auto-connect, if it fails, it starts AP mode with config portal
  bool res = wm.autoConnect("Setup-ESP32", "12345678");

  if (!res) {
    Serial.println("Failed to connect or timeout. Restart device.");
    delay(3000);
    ESP.restart();
  }

  Serial.println("Connected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  
  
  // Custom characters
  lcd.createChar(0, batteryIcon);  // Battery icon
  lcd.createChar(1, chargingIcon); // Charging icon
  
  // EEPROM & HX711
  EEPROM.begin(512);
  EEPROM.get(EEPROM_ADDR_CAL, calibrationValue);
  loadCell.begin();
  loadCell.start(2000, true);
  if (loadCell.getTareTimeoutFlag()) delay(1000);
  loadCell.setCalFactor(calibrationValue);
  
  // Wake on capture button
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_12, 0);
  
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("NT Scale");
  lcd.setCursor(0,1); lcd.print("Weight: ");
  lcd.setCursor(0,3);
  lcd.print(autoCaptureEnabled?"Mode: Auto":"Mode: Manual");

  lastActivityTime = millis();
  for (int i = 0; i < stable_window; i++) weightBuffer[i] = 0.0f;
}

int test = 0;

// ----------------------------------------------------------------------------
// Main loop
// ----------------------------------------------------------------------------
void loop(){
  // Calibration & Tare
  if (digitalRead(CALIBRATE_BTN) == LOW
      && millis() - lastCalBtnTime > DEBOUNCE_MS) {
    lastCalBtnTime = millis();
    handleCalibration();
  }
  
  if (digitalRead(TARE_BTN) == LOW
      && millis() - lastTareBtnTime > DEBOUNCE_MS) {
    lastTareBtnTime = millis();
    handleTare();
  }

  if (digitalRead(RESET_BTN) == LOW) {
    while(digitalRead(RESET_BTN) == LOW); // wait for release
    autoCaptureEnabled = !autoCaptureEnabled;
    lcd.setCursor(0,3);
    lcd.print("                   "); // Clear line
    lcd.setCursor(0,3);
    lcd.print(autoCaptureEnabled?"Mode: Auto":"Mode: Manual");
  }

  // Weigh (auto or manual)
  float eventWeight = autoCaptureEnabled
                      ? autoCaptureWeight()
                      : manualCaptureWeight();

  // Display capture on line 1, else live
  if (eventWeight > 0) {
    if (!capturingDisplayed) {
      lcd.setCursor(0,1);
      lcd.print("Weight:       ");
      lcd.setCursor(8,1);
      lcd.print(lastCapturedWeight,2);
      lcd.print("kg");
      capturingDisplayed = true;
    }
    lastActivityTime = millis();
  } else {
    // if we were displaying "Captured:" and now in manual mode the chicken is off
    if (!autoCaptureEnabled && capturingDisplayed) {
      float raw = loadCell.getData() / 1000.0f;
      while (raw > presence_threshold){
        // read a fresh weight
        loadCell.update();
        raw = loadCell.getData() / 1000.0f;
        delay(200);
      }
      // chicken is off → go back to live‐weight header
      lcd.setCursor(0,0); 
      lcd.print("NT Scale");
      lcd.setCursor(0,1); 
      lcd.print("Weight: ");
      lcd.setCursor(0,2);
      lcd.print("                   ");
      lcd.setCursor(0,3);  
      lcd.print(autoCaptureEnabled?"Mode: Auto":"Mode: Manual");
      lastActivityTime = millis();
      capturingDisplayed = false;
    }
    // now show live weight (will only redraw the number)
    showLiveWeight();
  }

  // Periodic battery/status update
  if (millis() - lastPrintTime > printInterval) {
    lastPrintTime = millis();
    readBattery();
  }

  // Inactivity → deep-sleep
  if (millis() - lastActivityTime >= IDLE_TOUT) {
    prepareDeepSleep();
  }
}