#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_NeoPixel.h>

// WS2812 LED setup
#define LED_PIN    13
#define NUM_LEDS   4
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// LED indices
#define LEFT_BLINDSPOT_LED       0  // Left ultrasonic
#define HB100X_LED               1  // HB100X
#define REAR_JSN_ULTRASONIC_LED  2  // Rear ultrasonic
#define RIGHT_BLINDSPOT_LED      3  // Right ultrasonic

// Serial UART communication to Arduino Nano
#define TX_PIN 17
#define RX_PIN 16

// LED timing
unsigned long lastBlinkTime = 0;
unsigned long lastHB100XBlinkTime = 0;
bool blinkState = false;
bool hb100xBlinkState = false;
const unsigned long BLINK_INTERVAL = 500;  // 500ms for fast blink
const unsigned long SLOW_BLINK_INTERVAL = 1000;  // 1000ms for slow blink

// Incoming data structure (matches sender format)
typedef struct sensor_data_message {
  uint8_t hb100x_value;           // 3-bit
  uint8_t ultrasonic_rear_value;  // 3-bit
  uint8_t ultrasonic_left_value;  // 1-bit
  uint8_t ultrasonic_right_value; // 1-bit
  bool is_danger;                 // critical condition flag
} sensor_data_message;

sensor_data_message incomingData;
unsigned long lastReceiveTime = 0;

// Stage-based sound command to Arduino
void sendSoundCommand(int stage) {
    if (stage == 0) {
    Serial2.write(0);  // Stop sound
    Serial.print("🔊 Sound stopped (Safe zone)\n");
  } else {
    Serial2.write(stage);
    Serial.print("🔊 Sent sound stage: ");
    Serial.println(stage);
  }
}

// Print sensor data for debugging
void printReceivedData() {
  Serial.println("Data Received:");
  Serial.print("  HB100X: "); Serial.println(incomingData.hb100x_value, BIN);
  Serial.print("  Rear:   "); Serial.println(incomingData.ultrasonic_rear_value, BIN);
  Serial.print("  Left:   "); Serial.println(incomingData.ultrasonic_left_value);
  Serial.print("  Right:  "); Serial.println(incomingData.ultrasonic_right_value);
  Serial.print("  Danger: "); Serial.println(incomingData.is_danger ? "YES" : "NO");
  Serial.println();
}

// Handle rear ultrasonic LED patterns
void handleRearUltrasonicLED() {
  unsigned long currentTime = millis();
  
  switch(incomingData.ultrasonic_rear_value) {
    case 0b000:  // Clear (no object)
      strip.setPixelColor(REAR_JSN_ULTRASONIC_LED, strip.Color(0, 255, 0));
      break;
      
    case 0b001:  // Very safe
      strip.setPixelColor(REAR_JSN_ULTRASONIC_LED, strip.Color(0, 255, 0));
      break;
      
    case 0b010:  // Safe
      strip.setPixelColor(REAR_JSN_ULTRASONIC_LED, strip.Color(0, 255, 0));
      break;
      
    case 0b011:  // Caution
      if (currentTime - lastBlinkTime >= SLOW_BLINK_INTERVAL) {
        blinkState = !blinkState;
        lastBlinkTime = currentTime;
        strip.setPixelColor(REAR_JSN_ULTRASONIC_LED, blinkState ? strip.Color(255, 255, 0) : 0);
      }
      break;
      
    case 0b100:  // Low warning
      strip.setPixelColor(REAR_JSN_ULTRASONIC_LED, strip.Color(255, 255, 0));
      break;
      
    case 0b101:  // Medium warning
      if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
        blinkState = !blinkState;
        lastBlinkTime = currentTime;
        strip.setPixelColor(REAR_JSN_ULTRASONIC_LED, blinkState ? strip.Color(255, 255, 0) : 0);
      }
      break;
      
    case 0b110:  // High warning
      if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
        blinkState = !blinkState;
        lastBlinkTime = currentTime;
        strip.setPixelColor(REAR_JSN_ULTRASONIC_LED, blinkState ? strip.Color(255, 0, 0) : 0);
      }
      break;
      
    case 0b111:  // no reading
      strip.setPixelColor(REAR_JSN_ULTRASONIC_LED, 0);
      break;
  }
}

// Handle HB100X LED patterns
void handleHB100XLED() {
  unsigned long currentTime = millis();
  
  switch(incomingData.hb100x_value) {
    case 0b000:  // No speed
      strip.setPixelColor(HB100X_LED, 0);
	sendSoundCommand(0);
      break;
      
    case 0b001:  // 2-5 km/h
      strip.setPixelColor(HB100X_LED, strip.Color(255, 255, 0));
	sendSoundCommand(0);
      break;
      
    case 0b010:  // 5-10 km/h
      if (currentTime - lastHB100XBlinkTime >= SLOW_BLINK_INTERVAL) {
        hb100xBlinkState = !hb100xBlinkState;
        lastHB100XBlinkTime = currentTime;
        strip.setPixelColor(HB100X_LED, hb100xBlinkState ? strip.Color(255, 255, 0) : 0);
	sendSoundCommand(1);
      }
      break;
      
    case 0b011:  // 10-15 km/h
      if (currentTime - lastHB100XBlinkTime >= BLINK_INTERVAL) {
        hb100xBlinkState = !hb100xBlinkState;
        lastHB100XBlinkTime = currentTime;
        strip.setPixelColor(HB100X_LED, hb100xBlinkState ? strip.Color(255, 255, 0) : 0);
	sendSoundCommand(1);
      }
      break;
      
    case 0b100:  // 15-20 km/h
      strip.setPixelColor(HB100X_LED, strip.Color(255, 0, 0));
	sendSoundCommand(2);
      break;
      
    case 0b101:  // 20-25 km/h
      if (currentTime - lastHB100XBlinkTime >= SLOW_BLINK_INTERVAL) {
        hb100xBlinkState = !hb100xBlinkState;
        lastHB100XBlinkTime = currentTime;
        strip.setPixelColor(HB100X_LED, hb100xBlinkState ? strip.Color(255, 0, 0) : 0);
	sendSoundCommand(2);
      }
      break;
      
    case 0b110:  // 25-30 km/h
      if (currentTime - lastHB100XBlinkTime >= BLINK_INTERVAL) {
        hb100xBlinkState = !hb100xBlinkState;
        lastHB100XBlinkTime = currentTime;
        strip.setPixelColor(HB100X_LED, hb100xBlinkState ? strip.Color(255, 0, 0) : 0);
	sendSoundCommand(3);
      }
      break;
      
    case 0b111:  // >30 km/h
      if (currentTime - lastHB100XBlinkTime >= BLINK_INTERVAL) {
        hb100xBlinkState = !hb100xBlinkState;
        lastHB100XBlinkTime = currentTime;
        strip.setPixelColor(HB100X_LED, hb100xBlinkState ? strip.Color(255, 0, 0) : 0);
	sendSoundCommand(4);
      }
      break;
  }
}

// Main warning handler
void processWarning() {
  // Handle HB100X LED
  handleHB100XLED();

  // Handle rear ultrasonic LED
  handleRearUltrasonicLED();

  // Left & Right Blindspot - Green only
  strip.setPixelColor(LEFT_BLINDSPOT_LED, incomingData.ultrasonic_left_value ? strip.Color(0, 255, 0) : 0);
  strip.setPixelColor(RIGHT_BLINDSPOT_LED, incomingData.ultrasonic_right_value ? strip.Color(0, 255, 0) : 0);

  // Send HB100X data to Arduino Nano
  String hb100xData = "H:" + String(incomingData.hb100x_value, BIN) + "\n";
  Serial2.print(hb100xData);

  strip.show();
}

// ESP-NOW receive callback for ESP32 Arduino core 3.x+
void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *incoming, int len) {
  memcpy(&incomingData, incoming, sizeof(incomingData));
  lastReceiveTime = millis();
  printReceivedData();
  processWarning();
}

void setup() {
  Serial.begin(115200);       // Debug serial
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);  // UART to Arduino Nano

  strip.begin();
  strip.show();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println(" ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onDataReceive);
  Serial.println(" ESP32#3 Ready.");
}

void loop() {
  if (millis() - lastReceiveTime > 5000) {
    Serial.println("No data received. Check connection!");
  }
  // Update LED states without delay
  handleHB100XLED();
  handleRearUltrasonicLED();
  strip.show();
}
