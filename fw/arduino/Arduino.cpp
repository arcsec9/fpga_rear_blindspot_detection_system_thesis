#include <Arduino.h>

// Pin connected to speaker
const int SPEAKER_PIN = 9;

// Timing variables for beep duration control
unsigned long lastCommandTime = 0;
const unsigned long BEEP_TIMEOUT = 3000; // 3 seconds timeout
bool beepActive = false;
int currentStage = 0;

// Chime sound pattern (imitating Toyota seatbelt chime)
void playChime(int repeatCount) {
  for (int i = 0; i < repeatCount; i++) {
    tone(SPEAKER_PIN, 1000, 150);  // 1 kHz tone for 150ms
    delay(200);                    // Pause between tones
    tone(SPEAKER_PIN, 1200, 100);  // 1.2 kHz tone for 100ms
    delay(150);
  }
}

// Danger stages mapped to different sound patterns
void playStageSound(int stage) {
  switch (stage) {
    case 1: // Low speed — single chime
      playChime(1);
      break;
    case 2: // Moderate — double chime
      playChime(2);
      break;
    case 3: // High — triple chime
      playChime(3);
      break;
    case 4: // Critical — continuous chimes (4x)
      playChime(4);
      break;
    default:
      // No sound
      break;
  }
  
  // Update timing variables
  lastCommandTime = millis();
  beepActive = true;
  currentStage = stage;
}

void setup() {
  Serial.begin(9600);       // Serial UART from ESP32#3
  pinMode(SPEAKER_PIN, OUTPUT);
}

void loop() {
  // Check for new commands
  if (Serial.available()) {
    int stage = Serial.read();  // Receive stage value from ESP32
    if (stage >= 1 && stage <= 4) {
      playStageSound(stage);
    } else if (stage == 0) {
      // Explicit command to stop beeping
      beepActive = false;
      currentStage = 0;
    }
  }
  
  // Check if we need to time out the beeping
  if (beepActive && (millis() - lastCommandTime >= BEEP_TIMEOUT)) {
    beepActive = false;
    currentStage = 0;
    // No need to explicitly stop tone as it's not continuously playing
  }
  
  // Optionally repeat the sound while active for continuous feedback
  // Only for critical alerts (stage 4)
  if (beepActive && currentStage == 4 && (millis() - lastCommandTime >= 1000)) {
    // For critical alerts, repeat every second while within the 3-second window
    playChime(4);
  }
}