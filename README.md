# Real-Time Non-Invasive Vehicle Rear and Blind-Spot Early Collision Detection System using FPGA, Microwave Radar, and Ultrasonic Sensors
An undergraduate thesis developed by Kurt Liam Santillan and Lian Miguel Jimena

## Components used:
3 ESP32 38-pin dev module,
2 Lux Sensor TSL2561,
3 JSN-SR04T Ultrasonic,
1 Cyclone IV EP4CE6E22C8N,
1 HB100x,
5 LEDs,
1 Beeper

---

## Data Path
Sensors -> ESP32_1 -> ESP32_2 -> FPGA -> ESP32_3 -> Output   

---

## Includes:
 - [Wire.h](https://github.com/espressif/arduino-esp32/blob/master/libraries/Wire/src/Wire.h)
 - [Adafruit_Sensor.h](https://github.com/adafruit/adafruit_sensor)
 - [Adafruit_TSL2561_U.h](https://github.com/adafruit/Adafruit_TSL2561/blob/master/Adafruit_TSL2561_U.h)
 - [esp_now.h](https://github.com/espressif/esp-idf/blob/master/components/esp_wifi/include/esp_now.h)
 - [Wifi.h](https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/src/WiFi.h)
 - [Adafruit_NeoPixel.h](https://github.com/adafruit/adafruit_neopixel)

---