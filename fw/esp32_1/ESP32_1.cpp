#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

// Input from sensors pin definitions
#define TRIG_LEFT 13   
#define ECHO_LEFT 14   
#define TRIG_RIGHT 4   
#define ECHO_RIGHT 5   
#define TRIG_REAR 12   
#define ECHO_REAR 15   

// FPGA interface pin definitions (10 pins for data output to FPGA)
#define DATA_PIN_0 16
#define DATA_PIN_1 17
#define DATA_PIN_2 18
#define DATA_PIN_3 19
#define DATA_PIN_4 23
#define DATA_PIN_5 25
#define DATA_PIN_6 26
#define DATA_PIN_7 27
#define DATA_PIN_8 32
#define DATA_PIN_9 33

#define SENSOR_SELECT_0 39  // Input from FPGA for sensor selection
#define SENSOR_SELECT_1 36

#define DATA_READY 35  // Input from FPGA requesting data

#define STROBE_PIN 2   // Output to FPGA indicating data is ready to read

// Debug control - set to false to disable FPGA request messages
#define DEBUG_FPGA_REQUESTS false
// Debug lux sensors - set to false in production for speed
#define DEBUG_LUX_SENSORS false

// Faster I2C clock speed
#define I2C_CLOCK_SPEED 800000  // Increased from 400000 to 800000

const float REAR_CALIBRATION_FACTOR = 1.0;

// HB100X variables
float currentFrequency = 0;
const int hb100xPin = 34;  // HB100 IF output
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
unsigned long lastMeasurementTime = 0;
float currentSpeed = 0;
float filteredSpeed = 0;
bool hb100Active = false;

// Lux sensor parameters - optimized for faster response
const int LUX_THRESHOLD = 30;
const unsigned long LUX_GRACE_PERIOD = 4000;  // Reduced from 2000ms to 1000ms
const unsigned long LUX_READ_INTERVAL = 100;    // Increased from 50ms to 100ms

// Default lux values (assume light is on if sensors fail)
uint32_t lux_left = 100;
uint32_t lux_right = 100;
bool luxSensorsWorking = false;

// Filtering constants
float filtered_left = 0.0;
float filtered_right = 0.0;
float filtered_rear = 0.0;

// JSN-SR04T specific parameters
const int MIN_VALID_DISTANCE = 25;    // JSN-SR04T minimum range
const int MAX_VALID_DISTANCE = 500;   // Extended to 5 meters
const unsigned long ULTRASONIC_TIMEOUT = 30000; // 30ms timeout for longer range
const int TRIGGER_PULSE_WIDTH = 15;   // 15μs trigger pulse for better range

// Rear sensor specific parameters
const int REAR_MAX_JUMP = 20;     // 20cm maximum jump between readings

// Error detection parameters
const int MAX_CONSECUTIVE_ERRORS = 5;  // Increased from 3 to 5
const int MIN_STABLE_READINGS = 10;  // Increased from 5 to 10

// Sensor stability tracking
unsigned long lastValidRearTime = 0;
unsigned long lastValidLeftTime = 0;
unsigned long lastValidRightTime = 0;
int consecutiveRearErrors = 0;
int consecutiveLeftErrors = 0;
int consecutiveRightErrors = 0;
int stableRearReadings = 0;
int stableLeftReadings = 0;
int stableRightReadings = 0;
float lastStableRearReading = 0;
float lastStableLeftReading = 0;
float lastStableRightReading = 0;

// Temperature compensation for ultrasonic (speed of sound varies with temperature)
const float TEMPERATURE = 28.1;  // Bacolod April Temperature
const float SPEED_OF_SOUND = 331.3 + (0.606 * TEMPERATURE);  // Speed of sound in m/s

// JSN-SR04T specific calibration factors
const float LEFT_CALIBRATION_FACTOR = 1.0;    // Default calibration
const float RIGHT_CALIBRATION_FACTOR = 1.0;   // Default calibration

// HB100X specific parameters
const float HB100_CALIBRATION = 0.05134;  // Calibration factor for speed calculation
const float HB100_MIN_FREQUENCY = 5.0;   // Increased minimum frequency threshold
const float HB100_MAX_FREQUENCY = 3000.0; // Reduced maximum frequency threshold
const float HB100_NOISE_THRESHOLD = 0.2;  // Increased noise threshold

// HB100X timing parameters
const unsigned long HB100_INTERVAL = 50;  // Reduced from 100ms to 50ms for 20Hz update rate
const unsigned long HB100_TIMEOUT = 1000;  // Reduced from 2000ms to 1000ms

// HB100X state tracking
Adafruit_TSL2561_Unified tsl_left = Adafruit_TSL2561_Unified(0x29);
Adafruit_TSL2561_Unified tsl_right = Adafruit_TSL2561_Unified(0x39);

// Grace period tracking
unsigned long leftLuxBelowTimestamp = 0;
bool leftLuxBelow = false;

unsigned long rightLuxBelowTimestamp = 0;
bool rightLuxBelow = false;

// Communication monitoring
unsigned long lastFpgaRequestTime = 0;
unsigned long fpgaRequestCount = 0;
bool fpgaCommActive = false;

// Debug values for threshold troubleshooting
int lastHb100Value = 0;
int lastRearValue = 0;
int lastLeftValue = 0;
int lastRightValue = 0;

// Sensor reading timing - optimized for better performance
unsigned long lastSensorReadTime = 0;
const unsigned long SENSOR_READ_INTERVAL = 5;  // Reduced from 10ms to 5ms for 200Hz update rate

// Lux sensor reading timing
unsigned long lastLuxReadTime = 0;

// Ultrasonic sensor stability tracking
const unsigned long STABILITY_TIMEOUT = 10; // Reduced from 30ms to 10ms

// Serial output timing - reduced frequency to improve performance
unsigned long lastSerialOutputTime = 0;
const unsigned long SERIAL_OUTPUT_INTERVAL = 2000; // Increased from 1000ms to 2000ms

// FPGA communication timing
const unsigned long FPGA_REQUEST_INTERVAL = 5;  // 5ms for 200Hz update rate

// Function prototypes
int measureDistance(int trigPin, int echoPin, float calibrationFactor);
void sendDataToFPGA(int value);
void readLuxSensors();
void readUltrasonicSensors();
void readHB100Sensor();
void handleFPGARequest();
void printSensorData();

void IRAM_ATTR countPulse() {
  unsigned long currentTime = micros();
  // Minimal debounce time for real-time response
  if (currentTime - lastPulseTime > 20) {
    pulseCount++;
    lastPulseTime = currentTime;
  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C with faster clock speed
  Wire.begin();
  Wire.setClock(I2C_CLOCK_SPEED);

  // Original sensor pin setup
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_REAR, OUTPUT);
  pinMode(ECHO_REAR, INPUT);
  pinMode(hb100xPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(hb100xPin), countPulse, RISING);

  // FPGA interface pin setup - using direct port manipulation for speed
  pinMode(DATA_PIN_0, OUTPUT);
  pinMode(DATA_PIN_1, OUTPUT);
  pinMode(DATA_PIN_2, OUTPUT);
  pinMode(DATA_PIN_3, OUTPUT);
  pinMode(DATA_PIN_4, OUTPUT);
  pinMode(DATA_PIN_5, OUTPUT);
  pinMode(DATA_PIN_6, OUTPUT);
  pinMode(DATA_PIN_7, OUTPUT);
  pinMode(DATA_PIN_8, OUTPUT);
  pinMode(DATA_PIN_9, OUTPUT);
  
  pinMode(SENSOR_SELECT_0, INPUT);
  pinMode(SENSOR_SELECT_1, INPUT);
  
  pinMode(DATA_READY, INPUT);
  pinMode(STROBE_PIN, OUTPUT);
  digitalWrite(STROBE_PIN, LOW);
  
  // Initialize light sensors with multiple attempts
  bool tsl_left_ok = false;
  bool tsl_right_ok = false;
  
  // Try to initialize the sensors a few times
  for (int attempt = 0; attempt < 3; attempt++) {
    if (!tsl_left_ok) tsl_left_ok = tsl_left.begin();
    if (!tsl_right_ok) tsl_right_ok = tsl_right.begin();
    
    if (tsl_left_ok && tsl_right_ok) break;
    delay(20); // Reduced delay
  }
  
  if (!tsl_left_ok || !tsl_right_ok) {
    luxSensorsWorking = false;
    // Set initial lux values to trigger ultrasonic activation
    lux_left = 0;  // Force ultrasonic activation
    lux_right = 0;
    leftLuxBelow = true;
    rightLuxBelow = true;
  } else {
    luxSensorsWorking = true;
    
    // Configure the sensors - using fastest integration time
    tsl_left.enableAutoRange(true);
    tsl_left.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
    tsl_right.enableAutoRange(true);
    tsl_right.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
    
    // Initial reading to verify they're working
    uint16_t broadband_left = 0, ir_left = 0, broadband_right = 0, ir_right = 0;
    
    // getLuminosity doesn't return a value, so we just call it
    tsl_left.getLuminosity(&broadband_left, &ir_left);
    tsl_right.getLuminosity(&broadband_right, &ir_right);
    
    // Check if we got reasonable values
    if (broadband_left > 0 || ir_left > 0 || broadband_right > 0 || ir_right > 0) {
      lux_left = tsl_left.calculateLux(broadband_left, ir_left) / 64;
      lux_right = tsl_right.calculateLux(broadband_right, ir_right) / 64;
      leftLuxBelow = (lux_left <= LUX_THRESHOLD);
      rightLuxBelow = (lux_right <= LUX_THRESHOLD);
    } else {
      luxSensorsWorking = false;
      // Set initial lux values to trigger ultrasonic activation
      lux_left = 0;
      lux_right = 0;
      leftLuxBelow = true;
      rightLuxBelow = true;
    }
  }

  // Initialize ultrasonic sensors with immediate readings
  filtered_left = measureDistance(TRIG_LEFT, ECHO_LEFT, LEFT_CALIBRATION_FACTOR);
  filtered_right = measureDistance(TRIG_RIGHT, ECHO_RIGHT, RIGHT_CALIBRATION_FACTOR);
  filtered_rear = measureDistance(TRIG_REAR, ECHO_REAR, REAR_CALIBRATION_FACTOR);
  
  // Initialize timestamps for grace period
  leftLuxBelowTimestamp = millis();
  rightLuxBelowTimestamp = millis();

  Serial.println("System Initialized ");
}

void loop() {
  // Check FPGA first for fastest response
  if (digitalRead(DATA_READY) == HIGH) {
    handleFPGARequest();
  }
  
  // Read sensors at regular intervals
  if (millis() - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    readUltrasonicSensors();
    readHB100Sensor();
    lastSensorReadTime = millis();
  }
  
  // Read lux sensors less frequently (they're slow)
  if (luxSensorsWorking && millis() - lastLuxReadTime >= LUX_READ_INTERVAL) {
    readLuxSensors();
    lastLuxReadTime = millis();
  }
  
  // Check if FPGA communication has stopped (only print once)
  static bool commLostReported = false;
  if (fpgaCommActive && (millis() - lastFpgaRequestTime > 1000)) {  // Reduced from 2000ms
    if (!commLostReported) {
      Serial.println("⚠️ FPGA communication lost! Last request was 1+ second ago.");
      commLostReported = true;
    }
    fpgaCommActive = false;
  } else if (fpgaCommActive) {
    commLostReported = false;
  }
  
  // Print sensor data less frequently to improve performance
  if (millis() - lastSerialOutputTime >= SERIAL_OUTPUT_INTERVAL) {
    printSensorData();
    lastSerialOutputTime = millis();
  }
}

// Separated FPGA handling for better organization and speed
void handleFPGARequest() {
  // Track FPGA communication
  fpgaRequestCount++;
  lastFpgaRequestTime = millis();
  fpgaCommActive = true;
  
  // Read sensor selection from FPGA
  uint8_t sensorSelect = digitalRead(SENSOR_SELECT_0) | (digitalRead(SENSOR_SELECT_1) << 1);
  
  // Prepare data based on sensor selection
  int sensorValue = 0;
  
  switch(sensorSelect) {
    case 0: // HB100X (00)
      sensorValue = constrain(map(filteredSpeed * 100, 0, 5250, 0, 1023), 0, 1023);
      lastHb100Value = sensorValue;
      break;
      
    case 1: // Rear ultrasonic (01)
      if (filtered_rear > 0) {
        sensorValue = constrain(map(filtered_rear, 0, 500, 0, 1023), 0, 1023);
        lastRearValue = sensorValue;
      }
      break;
      
    case 2: // Left ultrasonic (10)
      if (filtered_left > 0) {
        sensorValue = constrain(map(filtered_left, 0, 500, 0, 1023), 0, 1023);
        lastLeftValue = sensorValue;
      }
      break;
      
    case 3: // Right ultrasonic (11)
      if (filtered_right > 0) {
        sensorValue = constrain(map(filtered_right, 0, 500, 0, 1023), 0, 1023);
        lastRightValue = sensorValue;
      }
      break;
  }
  
  // Send data to FPGA
  sendDataToFPGA(sensorValue);
  
  // Pulse strobe to indicate data is ready for FPGA to read
  digitalWrite(STROBE_PIN, HIGH);
  delayMicroseconds(2); // Reduced from 10us
  digitalWrite(STROBE_PIN, LOW);
  
  // Only print FPGA requests if debug is enabled and only every 500th request
  if (DEBUG_FPGA_REQUESTS && fpgaRequestCount % 500 == 0) {
    Serial.printf("FPGA req #%lu | Sensor: %d | Value: %d\n", 
                  fpgaRequestCount, sensorSelect, sensorValue);
  }
}

void readLuxSensors() {
  if (!luxSensorsWorking) return;
  
  uint16_t broadband_left = 0, ir_left = 0, broadband_right = 0, ir_right = 0;
  
  tsl_left.getLuminosity(&broadband_left, &ir_left);
  tsl_right.getLuminosity(&broadband_right, &ir_right);
  
  bool valid_readings = (broadband_left > 0 || ir_left > 0) && 
                       (broadband_right > 0 || ir_right > 0);
  
  if (valid_readings) {
    uint32_t new_lux_left = tsl_left.calculateLux(broadband_left, ir_left) / 64;
    uint32_t new_lux_right = tsl_right.calculateLux(broadband_right, ir_right) / 64;
    
    if (new_lux_left <= 1000 && new_lux_right <= 1000) {
      lux_left = new_lux_left;
      lux_right = new_lux_right;
    }
    
    // Immediate activation when threshold is crossed
    if (lux_left > LUX_THRESHOLD) {
      if (leftLuxBelow) {
        // Force immediate ultrasonic activation
        filtered_left = -1;  // Reset to force new reading
        leftLuxBelow = false;
        leftLuxBelowTimestamp = 0;
        // Take immediate ultrasonic reading
        int raw_left = measureDistance(TRIG_LEFT, ECHO_LEFT, LEFT_CALIBRATION_FACTOR);
        if (raw_left >= MIN_VALID_DISTANCE && raw_left <= MAX_VALID_DISTANCE) {
          filtered_left = raw_left;
        }
      }
    } else {
      if (!leftLuxBelow) {
        leftLuxBelow = true;
        leftLuxBelowTimestamp = millis();
      }
    }
    
    if (lux_right > LUX_THRESHOLD) {
      if (rightLuxBelow) {
        // Force immediate ultrasonic activation
        filtered_right = -1;  // Reset to force new reading
        rightLuxBelow = false;
        rightLuxBelowTimestamp = 0;
        // Take immediate ultrasonic reading
        int raw_right = measureDistance(TRIG_RIGHT, ECHO_RIGHT, RIGHT_CALIBRATION_FACTOR);
        if (raw_right >= MIN_VALID_DISTANCE && raw_right <= MAX_VALID_DISTANCE) {
          filtered_right = raw_right;
        }
      }
    } else {
      if (!rightLuxBelow) {
        rightLuxBelow = true;
        rightLuxBelowTimestamp = millis();
      }
    }
  } else {
    // If reading failed, try to recover
    if (DEBUG_LUX_SENSORS) {
      Serial.printf("Lux read failed - Left BB:%d IR:%d | Right BB:%d IR:%d\n", 
                    broadband_left, ir_left, broadband_right, ir_right);
    }
    
    // Try to reinitialize the sensors
    Wire.begin();
    Wire.setClock(I2C_CLOCK_SPEED);
    delay(10);
    bool tsl_left_ok = tsl_left.begin();
    bool tsl_right_ok = tsl_right.begin();
    
    if (tsl_left_ok && tsl_right_ok) {
      tsl_left.enableAutoRange(true);
      tsl_left.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
      tsl_right.enableAutoRange(true);
      tsl_right.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
      if (DEBUG_LUX_SENSORS) {
        Serial.println("Lux sensors recovered");
      }
      luxSensorsWorking = true;
    } else {
      if (DEBUG_LUX_SENSORS) {
        Serial.println(" Failed to recover lux sensors");
      }
      luxSensorsWorking = false;
      
      // Default to assuming light is on (safer)
      lux_left = 100;
      lux_right = 100;
      leftLuxBelow = false;
      rightLuxBelow = false;
    }
  }
}

void readUltrasonicSensors() {
  // --- REAR sensor (always active) ---
  int raw_rear = measureDistance(TRIG_REAR, ECHO_REAR, REAR_CALIBRATION_FACTOR);
  
  // Enhanced error detection and validation
  if (raw_rear >= MIN_VALID_DISTANCE && raw_rear <= MAX_VALID_DISTANCE) {
    // Check for sudden jumps in readings
    if (filtered_rear > 0 && abs(raw_rear - filtered_rear) > REAR_MAX_JUMP) {
      consecutiveRearErrors++;
      if (consecutiveRearErrors >= MAX_CONSECUTIVE_ERRORS) {
        // Reset sensor state after too many errors
        filtered_rear = 0.0;
        consecutiveRearErrors = 0;
        stableRearReadings = 0;
        Serial.println("Rear sensor reset due to consecutive errors");
      }
    } else {
      // Valid reading
      consecutiveRearErrors = 0;
      filtered_rear = raw_rear;
      lastValidRearTime = millis();
      
      // Track stable readings
      if (abs(raw_rear - lastStableRearReading) <= REAR_MAX_JUMP) {
        stableRearReadings++;
        if (stableRearReadings >= MIN_STABLE_READINGS) {
          lastStableRearReading = raw_rear;
        }
      } else {
        stableRearReadings = 0;
      }
    }
  } else {
    // Invalid reading
    consecutiveRearErrors++;
    if (consecutiveRearErrors >= MAX_CONSECUTIVE_ERRORS) {
      filtered_rear = 0.0;
      consecutiveRearErrors = 0;
      stableRearReadings = 0;
      Serial.println("Rear sensor reset due to invalid readings");
    }
  }

  // --- LEFT sensor (lux gated with grace period) ---
  bool leftActive = !leftLuxBelow || (millis() - leftLuxBelowTimestamp <= LUX_GRACE_PERIOD);
  
  if (leftActive) {
    int raw_left = measureDistance(TRIG_LEFT, ECHO_LEFT, LEFT_CALIBRATION_FACTOR);
    
    if (raw_left >= MIN_VALID_DISTANCE && raw_left <= MAX_VALID_DISTANCE) {
      // Check for sudden jumps in readings
      if (filtered_left > 0 && abs(raw_left - filtered_left) > REAR_MAX_JUMP) {
        consecutiveLeftErrors++;
        if (consecutiveLeftErrors >= MAX_CONSECUTIVE_ERRORS) {
          filtered_left = 0.0;
          consecutiveLeftErrors = 0;
          stableLeftReadings = 0;
        }
      } else {
        consecutiveLeftErrors = 0;
        filtered_left = raw_left;
        lastValidLeftTime = millis();
        
        // Track stable readings
        if (abs(raw_left - lastStableLeftReading) <= REAR_MAX_JUMP) {
          stableLeftReadings++;
          if (stableLeftReadings >= MIN_STABLE_READINGS) {
            lastStableLeftReading = raw_left;
          }
        } else {
          stableLeftReadings = 0;
        }
      }
    } else {
      consecutiveLeftErrors++;
      if (consecutiveLeftErrors >= MAX_CONSECUTIVE_ERRORS) {
        filtered_left = 0.0;
        consecutiveLeftErrors = 0;
        stableLeftReadings = 0;
      }
    }
  } else {
    filtered_left = 0.0;
  }

  // --- RIGHT sensor (lux gated with grace period) ---
  bool rightActive = !rightLuxBelow || (millis() - rightLuxBelowTimestamp <= LUX_GRACE_PERIOD);
  
  if (rightActive) {
    int raw_right = measureDistance(TRIG_RIGHT, ECHO_RIGHT, RIGHT_CALIBRATION_FACTOR);
    
    if (raw_right >= MIN_VALID_DISTANCE && raw_right <= MAX_VALID_DISTANCE) {
      // Check for sudden jumps in readings
      if (filtered_right > 0 && abs(raw_right - filtered_right) > REAR_MAX_JUMP) {
        consecutiveRightErrors++;
        if (consecutiveRightErrors >= MAX_CONSECUTIVE_ERRORS) {
          filtered_right = 0.0;
          consecutiveRightErrors = 0;
          stableRightReadings = 0;
        }
      } else {
        consecutiveRightErrors = 0;
        filtered_right = raw_right;
        lastValidRightTime = millis();
        
        // Track stable readings
        if (abs(raw_right - lastStableRightReading) <= REAR_MAX_JUMP) {
          stableRightReadings++;
          if (stableRightReadings >= MIN_STABLE_READINGS) {
            lastStableRightReading = raw_right;
          }
        } else {
          stableRightReadings = 0;
        }
      }
    } else {
      consecutiveRightErrors++;
      if (consecutiveRightErrors >= MAX_CONSECUTIVE_ERRORS) {
        filtered_right = 0.0;
        consecutiveRightErrors = 0;
        stableRightReadings = 0;
      }
    }
  } else {
    filtered_right = 0.0;
  }
}

void readHB100Sensor() {
  unsigned long currentTime = millis();
  
  // Check if it's time to measure
  if (currentTime - lastMeasurementTime >= HB100_INTERVAL) {
    // Disable interrupt while reading
    detachInterrupt(digitalPinToInterrupt(hb100xPin));
    
    // Calculate frequency
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    lastMeasurementTime = currentTime;
    
    // Re-enable interrupt
    attachInterrupt(digitalPinToInterrupt(hb100xPin), countPulse, RISING);
    
    // Calculate frequency with averaging
    currentFrequency = (pulses * 1000.0) / HB100_INTERVAL;  // Hz
    
    // Debug output for HB100X
    static unsigned long lastDebugTime = 0;
    if (currentTime - lastDebugTime >= 500) {  // Print every 500ms
      Serial.printf("HB100X - Pulses: %lu, Freq: %.2f Hz, Speed: %.2f km/h\n", 
                   pulses, currentFrequency, currentFrequency * HB100_CALIBRATION);
      lastDebugTime = currentTime;
    }
    
    // Validate frequency range
    if (currentFrequency >= HB100_MIN_FREQUENCY && currentFrequency <= HB100_MAX_FREQUENCY) {
      // Calculate speed from frequency
      currentSpeed = currentFrequency * HB100_CALIBRATION;
      
      // Apply noise threshold
      if (currentSpeed >= HB100_NOISE_THRESHOLD) {
        // More stable filtering
        filteredSpeed = 0.7 * currentSpeed + 0.3 * filteredSpeed;
        hb100Active = true;
      } else {
        // Below noise threshold, consider as stopped
        currentSpeed = 0;
        filteredSpeed = 0;
      }
    } else {
      // Invalid frequency, consider as stopped
      currentSpeed = 0;
      filteredSpeed = 0;
    }
    
    // Check for timeout
    if (hb100Active && currentSpeed == 0) {
      if (currentTime - lastPulseTime > HB100_TIMEOUT) {
        hb100Active = false;
        filteredSpeed = 0;
      }
    }
  }
}

// Improved ultrasonic distance measurement for extended range
int measureDistance(int trigPin, int echoPin, float calibrationFactor = 1.0) {
  long duration;
  float distance;
  
  // Stronger trigger pulse for better range
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(TRIGGER_PULSE_WIDTH);
  digitalWrite(trigPin, LOW);

  // Longer timeout for extended range
  duration = pulseIn(echoPin, HIGH, ULTRASONIC_TIMEOUT);
  
  if (duration > 0) {
    // Convert to distance using temperature-compensated speed of sound
    distance = (duration * SPEED_OF_SOUND * calibrationFactor) / 20000.0;  // Convert to cm
    
    // Check if reading is within extended valid range
    if (distance >= MIN_VALID_DISTANCE && distance <= MAX_VALID_DISTANCE) {
      return (int)distance;
    }
  }
  
  return 0;  // Return 0 instead of -1 for invalid readings
}

// Optimized data sending to FPGA
void sendDataToFPGA(int value) {
  // Direct port manipulation for faster output
  digitalWrite(DATA_PIN_0, (value & 0x001) ? HIGH : LOW);
  digitalWrite(DATA_PIN_1, (value & 0x002) ? HIGH : LOW);
  digitalWrite(DATA_PIN_2, (value & 0x004) ? HIGH : LOW);
  digitalWrite(DATA_PIN_3, (value & 0x008) ? HIGH : LOW);
  digitalWrite(DATA_PIN_4, (value & 0x010) ? HIGH : LOW);
  digitalWrite(DATA_PIN_5, (value & 0x020) ? HIGH : LOW);
  digitalWrite(DATA_PIN_6, (value & 0x040) ? HIGH : LOW);
  digitalWrite(DATA_PIN_7, (value & 0x080) ? HIGH : LOW);
  digitalWrite(DATA_PIN_8, (value & 0x100) ? HIGH : LOW);
  digitalWrite(DATA_PIN_9, (value & 0x200) ? HIGH : LOW);
  
  // Pulse strobe with minimal delay
  digitalWrite(STROBE_PIN, HIGH);
  delayMicroseconds(2); // Reduced from 10us
  digitalWrite(STROBE_PIN, LOW);
}

// Separated serial output to reduce impact on timing
void printSensorData() {
  // Print general sensor readings
  Serial.printf("Left Lux: %d | Right Lux: %d | ", lux_left, lux_right);
  Serial.printf("Left Distance: %.1f cm | Right Distance: %.1f cm | Rear Distance: %.1f cm | ", 
                filtered_left, filtered_right, filtered_rear);

  // HB100X detailed print
  Serial.print("Measured Speed (km/h): ");
  Serial.print(filteredSpeed, 2);
  Serial.print(" | ");

  Serial.print("Time to Detect (s): ");
  float timeToDetect = (filteredSpeed > 0) ? (lastMeasurementTime - lastPulseTime) / 1000.0 : 0;
  Serial.print(timeToDetect, 2);
  Serial.print(" | ");

  Serial.print("HB100X Detection: ");
  Serial.println(hb100Active ? "Yes" : "No");

  // Optional FPGA request tracking
  Serial.printf("FPGA reqs: %lu\n", fpgaRequestCount);
}
