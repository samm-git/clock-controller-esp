/**
   The MIT License (MIT)

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
*/

#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Timezone.h>    // https://github.com/JChristensen/Timezone
#include <TimeLib.h>     // https://playground.arduino.cc/Code/Time
#include "SSD1306Wire.h" // SSD1306 OLED: https://github.com/ThingPulse/esp8266-oled-ssd1306
#include "OLEDDisplayUi.h"
#include "images.h"      // Include custom images

// motor controller CH0 pins
#define PIN_CH00 12
#define PIN_CH01 13
#define PIN_INIT 15

// display configuration
#define DISP_I2C 0x3c
#define DISP_SDA 5
#define DISP_SCL 4

// WIFI configuration
#define WIFI_SSID "myssid"
#define WIFI_KEY  "mypass"
static const char ntpServerName[] = "cz.pool.ntp.org";
unsigned int localPort = 8888;
WiFiUDP udp;

// Initialize the OLED display using Wire library
SSD1306Wire display(DISP_I2C, DISP_SDA, DISP_SCL);

OLEDDisplayUi ui( & display);

int screenW = 128;
int screenH = 64;
int clockCenterX = screenW / 2;
int clockCenterY = ((screenH - 16) / 2);
short state = 0;
int wifi_connected = 0;
int ntp_started = 0;
char console_text[256];
Preferences preferences;

// Central European Time definition
TimeChangeRule CEST = { "CEST", Last, Sun, Mar, 2, 120 }; // Central European Summer Time
TimeChangeRule CET =  { "CET ", Last, Sun, Oct, 3, 60  }; // Central European Standard Time
Timezone ClockTZ(CEST, CET);

// utility function for digital clock display: prints leading 0
String twoDigits(int digits) {
  if (digits < 10) {
    String i = '0' + String(digits);
    return i;
  } else {
    return String(digits);
  }
}

void clockOverlay(OLEDDisplay * display, OLEDDisplayUiState * state) {

}

void digitalClockFrame(OLEDDisplay * display, OLEDDisplayUiState * state, int16_t x, int16_t y) {
  time_t utc = now();
  time_t local_t = ClockTZ.toLocal(utc);
  String timenow = String(hour(local_t)) + ":" + twoDigits(minute(local_t)) + ":" + twoDigits(second(local_t));
  display -> setTextAlignment(TEXT_ALIGN_CENTER);
  display -> setFont(ArialMT_Plain_24);
  display -> drawString(clockCenterX + x, clockCenterY + y, timenow);
}

void digitalClockStateFrame(OLEDDisplay * display, OLEDDisplayUiState * oled_state, int16_t x, int16_t y) {
  char buf[16];
  char * timenow = formatState(abs(state), buf, 16);
  display -> setTextAlignment(TEXT_ALIGN_CENTER);
  display -> setFont(ArialMT_Plain_24);
  display -> drawString(clockCenterX + x, clockCenterY + y, timenow);
}

// This array keeps function pointers to all frames
// frames are the single views that slide in
FrameCallback frames[] = {
  digitalClockFrame,
  digitalClockStateFrame
};

// how many frames are there?
int frameCount = 2;

// Overlays are statically drawn on top of a frame eg. a clock
OverlayCallback overlays[] = {
  clockOverlay
};
int overlaysCount = 1;

void setup() {
  Serial.begin(115200);
  /* Read slave clock state from the EEPROM using Preferences lib.
     We have 12 * 60 (720) possible values
     in the clock and also we need to keep last used polarity as a sign
     This mean that valid values are from -721 to +721 excluding 0
  */
  preferences.begin("clock", false);
  state = preferences.getShort("state", -1);
  Serial.printf("Booting... Initial state is: %d\n", state);
  if (state < -721 || state > 721 || state == 0) {
    state = 1; // init clock on 12:00
    preferences.putShort("state", state);
  }

  pinMode(PIN_CH00, OUTPUT); // init GPIO to control clock motor
  pinMode(PIN_CH01, OUTPUT);
  pinMode(PIN_INIT, INPUT_PULLUP);

  // set UI FPS
  ui.setTargetFPS(30);

  // Customize the active and inactive symbol
  ui.setActiveSymbol(activeSymbol);
  ui.setInactiveSymbol(inactiveSymbol);

  ui.setIndicatorPosition(TOP);

  // Defines where the first frame is located in the bar.
  ui.setIndicatorDirection(LEFT_RIGHT);

  // could be SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
  ui.setFrameAnimation(SLIDE_LEFT);

  // Add frames
  ui.setFrames(frames, frameCount);

  // Add overlays
  ui.setOverlays(overlays, overlaysCount);

  // Initialising the UI will init the display too.
  ui.init();

  display.flipScreenVertically();
  int initState = digitalRead(PIN_INIT);

  // if init mode is on - state is set to 12:00 and pin must be unplugged when
  // slave is displaying this value
  if (initState == LOW) {
    Serial.println("Clock init mode started");
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawStringMaxWidth(0, 0, 128,
      "Init mode");
    display.setFont(ArialMT_Plain_10);

    display.drawStringMaxWidth(0, 18, 128,
      "Set clock to 12:00 and unplug when ready");
    display.display();
    state = 1;
    while (initState == LOW) {
      if (state > 0) { // move clock arrow once a second, save last polarity
        state = 1;
        digitalWrite(PIN_CH00, HIGH);
        digitalWrite(PIN_CH01, LOW);
      } else {
        state = -1;
        digitalWrite(PIN_CH00, LOW);
        digitalWrite(PIN_CH01, HIGH);
      }
      // invert polarity on next run
      state = state * -1;
      delay(800);
      digitalWrite(PIN_CH00, LOW);
      digitalWrite(PIN_CH01, LOW);
      delay(200);
      preferences.putShort("state", state)
      initState = digitalRead(PIN_INIT);
    }
    display.clear();
  }

  // connect to wifi
  sprintf(console_text, "Connecting to wifi (%s)", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_KEY);
}

/*-------- Move arrow and update state ----------*/

void fixState(short curr_state) {
  char buf[16], buf2[16];
  Serial.printf("changing state from %d [%s] to %d [%s])\n", state, formatState(abs(state), buf, 16), curr_state, formatState(curr_state, buf2, 16));
  if (state > 0) {
    state++;
    if (state > 721) state = 1;
    digitalWrite(PIN_CH00, HIGH);
    digitalWrite(PIN_CH01, LOW);
  } else {
    state--;
    if (state < -721) state = -1;
    digitalWrite(PIN_CH00, LOW);
    digitalWrite(PIN_CH01, HIGH);
  }
  // invert polarity on the next run
  state = state * -1;
  delay(800);
  digitalWrite(PIN_CH00, LOW);
  digitalWrite(PIN_CH01, LOW);
  preferences.putShort("state", state);
}

char * formatState(int mystate, char * buf, int bufsize) {
  mystate--;
  snprintf(buf, bufsize, "%d:%02d", mystate / 60, mystate - (mystate / 60) * 60);
  return buf;
}

/*-------- Main loop ----------*/
void loop() {
  // init mode, clock showing state and connecting to WIFI
  if (!wifi_connected) {
    display.clear();
    display.setFont(ArialMT_Plain_10);
    // display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawStringMaxWidth(0, 0, 128,
      console_text);
    display.display();
    if (WiFi.status() != WL_CONNECTED) {
      delay(10);
      return;
    }
    Serial.print("Connected, IP address: ");
    Serial.println(WiFi.localIP());
    wifi_connected = 1;
  }
  if (!ntp_started) {
    Serial.println("Starting UDP...");
    udp.begin(localPort);
    Serial.println("Waiting for sync");
    display.drawStringMaxWidth(0, 10, 128,
      "Waiting for NTP sync");
    display.display();
    setSyncProvider(getNtpTime);
    setSyncInterval(300); // sync with NTP once in a 5m
    if (timeStatus() == timeNotSet) {
      delay(10);
      return;
    }
    ntp_started = 1;
  }

  int remainingTimeBudget = ui.update();
  int curr_state = 0;
  if (remainingTimeBudget > 0) {
    time_t utc = now();
    time_t local_t = ClockTZ.toLocal(utc);
    int hour_12 = hour(local_t);
    if (hour_12 >= 12) hour_12 -= 12;
    // current 12h time in minutes, starting from 1
    short curr_state = hour_12 * 60 + minute(local_t) + 1;

    if (curr_state != abs(state)) {
      fixState(curr_state);
      delay(200); // cool down device :)
    }
    delay(remainingTimeBudget);
  }
}

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime() {
  IPAddress ntpServerIP; // NTP server's ip address

  while (udp.parsePacket() > 0); // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      udp.read(packetBuffer, NTP_PACKET_SIZE); // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 = (unsigned long) packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long) packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long) packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long) packetBuffer[43];
      return secsSince1900 - 2208988800UL;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress & address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011; // LI, Version, Mode
  packetBuffer[1] = 0; // Stratum, or type of clock
  packetBuffer[2] = 6; // Polling Interval
  packetBuffer[3] = 0xEC; // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}
