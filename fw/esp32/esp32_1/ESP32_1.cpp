#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <driver_esp32.cpp>

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
      Serial.println("FPGA communication lost! Last request was 1+ second ago.");
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
