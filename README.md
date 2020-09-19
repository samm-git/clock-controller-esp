# Slave clock controller based on ESP8266 hardware

## Requirements

### Software

 - [Arduino Timezone Library](https://github.com/JChristensen/Timezone)
 - [Arduino Time Library ](https://playground.arduino.cc/Code/Time)
 - [ThingPulse OLED SSD1306 Library](https://github.com/ThingPulse/esp8266-oled-ssd1306)
 
### Hardware
 
 - [ESP-WROOM-32 0.96" OLED ESP32 WIFI-BT Dual-mode 2.4GHz For Wemos D1 AP STA](https://www.ebay.com/itm/ESP-WROOM-32-0-96-OLED-ESP32-WIFI-BT-Dual-mode-2-4GHz-For-Wemos-D1-AP-STA-/332196121504)
 - [L298N motor driver module H-Bridge](https://www.instructables.com/id/Control-DC-and-stepper-motors-with-L298N-Dual-Moto/)
 - 12V 1A power supply
 
 ## How it works
 
- After startup ESP trying to connect to WIFI and get time from the NTP
- If time is synced - ESP compares it with slave time in the NVRAM and updates slave clock
- Code respects configured timezone and automatically synced to NTP every 5m
- There is a special "init mode" to sync hardware slave with controller
- At the moment we are using only one (first) channel of the L298N. Second could be used for the alarm or led highliting. 

## Init mode

As ESP has no any information about slave clock position - we need to sync them. To do this - connect PIN_INIT (15) pin to GND and restart ESP. It will move arrows every second. Wait until clock shows 12:00 AM and immediately unplug wire. Clock will be synced with ESP and will switch to normal mode. 