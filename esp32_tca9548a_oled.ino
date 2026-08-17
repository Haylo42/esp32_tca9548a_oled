/*
 * TCA9548A: countdown on OLED ch0, smiley on OLED ch1.
 *
 *   ESP32 3V3 -> mux VCC, both OLED VCC
 *   ESP32 GND -> mux GND, both OLED GND
 *   ESP32 21  -> mux SDA
 *   ESP32 22  -> mux SCL
 *   ESP32 4   -> mux RST
 *   mux SD0/SC0 -> countdown OLED SDA/SCL
 *   mux SD1/SC1 -> smiley OLED SDA/SCL
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

static const int MUX_RESET_PIN = 4;
static const uint8_t OLED_ADDR = 0x3C;
static const uint8_t CH_COUNTDOWN = 0;
static const uint8_t CH_SMILEY = 1;
static const int SCREEN_WIDTH = 128;
static const int SCREEN_HEIGHT = 64;
static const int COUNTDOWN_START = 10;
static const uint32_t STEP_MS = 1000;

Adafruit_SH1106G displayCount(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_SH1106G displaySmile(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

static int gSda = 21;
static int gScl = 22;
static uint8_t gMuxAddr = 0;
static bool gCountReady = false;
static bool gSmileReady = false;

static const char *i2cErrName(uint8_t err) {
  switch (err) {
    case 0:
      return "ACK";
    case 2:
      return "NACK";
    case 5:
      return "TIMEOUT";
    default:
      return "OTHER";
  }
}

static uint8_t probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission();
}

static void startBus(int sda, int scl) {
  Wire.end();
  delay(5);
  Wire.begin(sda, scl);
  Wire.setTimeOut(50);
  Wire.setClock(50000);
}

static uint8_t scanForMux(int sda, int scl) {
  startBus(sda, scl);
  Serial.printf("SDA=%d SCL=%d\n", sda, scl);

  uint8_t foundMux = 0;
  Serial.println("  mux addresses:");
  for (uint8_t addr = 0x70; addr <= 0x77; ++addr) {
    const uint8_t err = probe(addr);
    Serial.printf("    0x%02X %s (%u)\n", addr, i2cErrName(err), err);
    if (err == 0 && foundMux == 0) {
      foundMux = addr;
    }
  }
  Serial.println("  scan done");
  return foundMux;
}

static bool selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(gMuxAddr);
  Wire.write(static_cast<uint8_t>(1 << channel));
  return Wire.endTransmission() == 0;
}

static bool initOled(Adafruit_SH1106G &d, uint8_t channel) {
  if (!selectMuxChannel(channel)) {
    Serial.printf("Mux channel %u select failed\n", channel);
    return false;
  }
  delay(10);
  if (probe(OLED_ADDR) != 0) {
    Serial.printf("No OLED 0x3C on channel %u\n", channel);
    return false;
  }
  if (!d.begin(OLED_ADDR, true)) {
    Serial.printf("SH1106 begin() failed on channel %u\n", channel);
    return false;
  }
  d.clearDisplay();
  d.display();
  Serial.printf("OLED ready on mux channel %u\n", channel);
  return true;
}

static void showNumber(int value) {
  selectMuxChannel(CH_COUNTDOWN);
  displayCount.clearDisplay();
  displayCount.setTextSize(4);
  displayCount.setTextColor(SH110X_WHITE);
  char buf[8];
  snprintf(buf, sizeof(buf), "%d", value);
  const int16_t textWidth = static_cast<int16_t>(strlen(buf) * 6 * 4);
  displayCount.setCursor((SCREEN_WIDTH - textWidth) / 2, (SCREEN_HEIGHT - 32) / 2);
  displayCount.print(buf);
  displayCount.display();
}

static void drawSmiley() {
  selectMuxChannel(CH_SMILEY);
  displaySmile.clearDisplay();

  const int16_t cx = SCREEN_WIDTH / 2;
  const int16_t cy = SCREEN_HEIGHT / 2;
  displaySmile.drawCircle(cx, cy, 28, SH110X_WHITE);
  displaySmile.drawCircle(cx, cy, 27, SH110X_WHITE);
  displaySmile.fillCircle(cx - 10, cy - 6, 4, SH110X_WHITE);
  displaySmile.fillCircle(cx + 10, cy - 6, 4, SH110X_WHITE);

  for (int16_t x = -14; x <= 14; ++x) {
    const int16_t y = (x * x) / 28;
    displaySmile.drawPixel(cx + x, cy + 16 - y, SH110X_WHITE);
    displaySmile.drawPixel(cx + x, cy + 17 - y, SH110X_WHITE);
  }

  displaySmile.display();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Mux: ch0 countdown, ch1 smiley");

  pinMode(MUX_RESET_PIN, OUTPUT);
  digitalWrite(MUX_RESET_PIN, LOW);
  delay(10);
  digitalWrite(MUX_RESET_PIN, HIGH);
  delay(20);

  gMuxAddr = scanForMux(21, 22);
  gSda = 21;
  gScl = 22;
  if (gMuxAddr == 0) {
    Serial.println("No mux on 21/22 — trying swapped 22/21");
    gMuxAddr = scanForMux(22, 21);
    gSda = 22;
    gScl = 21;
  }

  if (gMuxAddr == 0) {
    Serial.println("No TCA9548A found");
    return;
  }

  Serial.printf("Mux ACK at 0x%02X  SDA=%d SCL=%d\n", gMuxAddr, gSda, gScl);
  startBus(gSda, gScl);

  gCountReady = initOled(displayCount, CH_COUNTDOWN);
  gSmileReady = initOled(displaySmile, CH_SMILEY);

  if (gSmileReady) {
    drawSmiley();
  }
}

void loop() {
  if (!gCountReady) {
    delay(2000);
    return;
  }
  for (int n = COUNTDOWN_START; n >= 0; --n) {
    Serial.printf("countdown %d\n", n);
    showNumber(n);
    if (gSmileReady) {
      drawSmiley();
    }
    delay(STEP_MS);
  }
}
