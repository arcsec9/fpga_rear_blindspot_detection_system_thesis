#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Pin configuration for FPGA interface
const int HB100X_PINS[3] = {4, 5, 12};             // 3-bit HB100X
const int ULTRASONIC_REAR_PINS[3] = {14, 16, 17};  // 3-bit rear ultrasonic
const int ULTRASONIC_RIGHT_PIN = 18;              // 1-bit right ultrasonic
const int ULTRASONIC_LEFT_PIN = 23;               // 1-bit left ultrasonic

// Receiver MAC Address (ESP32#3)
uint8_t receiverMacAddress[] = {0x8C, 0x4F, 0x00, 0x2F, 0x96, 0x50};

// Structure to send
typedef struct sensor_data_message {
  uint8_t hb100x_value;           // 3-bit
  uint8_t ultrasonic_rear_value;  // 3-bit
  uint8_t ultrasonic_left_value;  // 1-bit
  uint8_t ultrasonic_right_value; // 1-bit
  bool is_danger;                 // critical condition flag
} sensor_data_message;

sensor_data_message sensorData;

// ESP-NOW send callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

// Add timing control variables
unsigned long lastTransmitTime = 0;
const unsigned long TRANSMIT_INTERVAL = 50;  // 50ms for 20Hz update rate

void setup() {
  Serial.begin(115200);

  // Set pin modes
  for (int i = 0; i < 3; i++) {
    pinMode(HB100X_PINS[i], INPUT);
    pinMode(ULTRASONIC_REAR_PINS[i], INPUT);
  }
  pinMode(ULTRASONIC_RIGHT_PIN, INPUT);
  pinMode(ULTRASONIC_LEFT_PIN, INPUT);

  // Set up ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ESP32#2 Ready. Reading FPGA data...");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  unsigned long currentTime = millis();
  
  // Only transmit at regular intervals
  if (currentTime - lastTransmitTime >= TRANSMIT_INTERVAL) {
    // Read HB100X 3-bit value
    uint8_t hb100x_value = 0;
    for (int i = 0; i < 3; i++) {
      if (digitalRead(HB100X_PINS[i])) {
        hb100x_value |= (1 << i);
      }
    }

    // Read rear ultrasonic 3-bit value
    uint8_t ultrasonic_rear_value = 0;
    for (int i = 0; i < 3; i++) {
      if (digitalRead(ULTRASONIC_REAR_PINS[i])) {
        ultrasonic_rear_value |= (1 << i);
      }
    }

    // Read left and right ultrasonic (1 bit each)
    uint8_t ultrasonic_left_value = digitalRead(ULTRASONIC_LEFT_PIN);
    uint8_t ultrasonic_right_value = digitalRead(ULTRASONIC_RIGHT_PIN);

    // Determine if there's danger
    bool dangerSpeed = (hb100x_value >= 0b110);            // ≥ 25.1 km/h
    bool dangerRear = (ultrasonic_rear_value >= 0b110);    // ≤ 1m
    bool dangerSide = (ultrasonic_left_value == 1 || ultrasonic_right_value == 1);
    bool is_danger = dangerSpeed || dangerRear || dangerSide;

    // Debug output (less frequent)
    static unsigned long lastDebugTime = 0;
    if (currentTime - lastDebugTime >= 1000) {  // Print every second
      Serial.println("\n📡 Sending Sensor Data:");
      Serial.print("  HB100X: "); Serial.println(hb100x_value, BIN);
      Serial.print("  Rear: "); Serial.println(ultrasonic_rear_value, BIN);
      Serial.print("  Left: "); Serial.println(ultrasonic_left_value);
      Serial.print("  Right: "); Serial.println(ultrasonic_right_value);
      Serial.print("  Danger: "); Serial.println(is_danger ? "YES" : "NO");
      lastDebugTime = currentTime;
    }

    // Fill structure
    sensorData.hb100x_value = hb100x_value;
    sensorData.ultrasonic_rear_value = ultrasonic_rear_value;
    sensorData.ultrasonic_left_value = ultrasonic_left_value;
    sensorData.ultrasonic_right_value = ultrasonic_right_value;
    sensorData.is_danger = is_danger;

    // Transmit to ESP32#3
    esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t*)&sensorData, sizeof(sensorData));
    if (result == ESP_OK) {
      lastTransmitTime = currentTime;
    } else {
      Serial.println("Send failed");
    }
  }
}
