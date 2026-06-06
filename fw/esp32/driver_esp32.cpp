#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <driver_esp32.h>

// Debug values for threshold troubleshooting
int lastHb100Value = 0;
int lastRearValue = 0;
int lastLeftValue = 0;
int lastRightValue = 0;

// Communication monitoring
unsigned long lastFpgaRequestTime = 0;
unsigned long fpgaRequestCount = 0;
bool fpgaCommActive = false;

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