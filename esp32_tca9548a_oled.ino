/*
 * Sweeping HC-SR04 on a 9g servo. Distance on ch0, eyes / ENEMY DETECTED on ch1.
 *
 * Alert is proximity: ENEMY DETECTED when something is closer than 20 cm.
 * The servo holds still while an enemy is in range, then resumes the sweep.
 *
 *   ESP32 18  -> servo signal (orange/yellow)
 *   ESP32 25  -> HC-SR04 TRIG
 *   ESP32 26  -> HC-SR04 ECHO (5 V Echo needs a divider)
 *   Servo VCC -> 5 V (VIN), NOT 3V3
 *   Servo GND -> ESP32 GND
 *   mux SD0/SC0 -> distance OLED
 *   mux SD1/SC1 -> eyes / enemy OLED
 */

#include <math.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <ESP32Servo.h>

static const int MUX_RESET_PIN = 4;
static const int TRIG_PIN = 25;
static const int ECHO_PIN = 26;
static const int SERVO_PIN = 18;
static const uint8_t OLED_ADDR = 0x3C;
static const uint8_t CH_DIST = 0;
static const uint8_t CH_FACE = 1;
static const int SCREEN_WIDTH = 128;
static const int SCREEN_HEIGHT = 64;

static const int SERVO_MIN = 40;
static const int SERVO_MAX = 140;
static const int SERVO_STEP = 1;
static const uint32_t SERVO_TICK_MS = 20;
static const uint32_t SERVO_DWELL_MS = 250;
static const uint32_t PING_PERIOD_MS = 120;
static const uint32_t BLINK_EVERY_MS = 2000;
static const uint32_t BLINK_CLOSED_MS = 180;
static const int ENEMY_RANGE_CM = 20;
static const unsigned long ECHO_TIMEOUT_US = 30000;

Adafruit_SH1106G displayDist(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_SH1106G displayGame(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Servo gServo;

static int gSda = 21;
static int gScl = 22;
static uint8_t gMuxAddr = 0;
static bool gDistReady = false;
static bool gGameReady = false;

static volatile int gAngle = SERVO_MIN;
static volatile int gDir = 1;
static volatile bool gEnemy = false;
static bool gEyesClosed = false;
static uint32_t gBlinkAtMs = 0;
static uint32_t gPingAtMs = 0;

static void servoTask(void * /*param*/) {
  uint32_t dwellLeftMs = 0;
  for (;;) {
    if (!gEnemy) {
      if (dwellLeftMs > 0) {
        gServo.write(gAngle);
        dwellLeftMs = (dwellLeftMs > SERVO_TICK_MS) ? (dwellLeftMs - SERVO_TICK_MS) : 0;
      } else {
        int angle = gAngle + gDir * SERVO_STEP;
        int dir = gDir;
        if (angle >= SERVO_MAX) {
          angle = SERVO_MAX;
          dir = -1;
          dwellLeftMs = SERVO_DWELL_MS;
        } else if (angle <= SERVO_MIN) {
          angle = SERVO_MIN;
          dir = 1;
          dwellLeftMs = SERVO_DWELL_MS;
        }
        gAngle = angle;
        gDir = dir;
        gServo.write(angle);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(SERVO_TICK_MS));
  }
}

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
  Wire.setClock(100000);
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

static int readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  const unsigned long us = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) {
    return -1;
  }
  return static_cast<int>(us / 58UL);
}

static void showDistance(int cm) {
  selectMuxChannel(CH_DIST);
  displayDist.clearDisplay();
  displayDist.setTextColor(SH110X_WHITE);

  char buf[8];
  if (cm < 0) {
    snprintf(buf, sizeof(buf), "---");
  } else {
    snprintf(buf, sizeof(buf), "%d", cm);
  }

  displayDist.setTextSize(3);
  const int16_t textWidth = static_cast<int16_t>(strlen(buf) * 6 * 3);
  displayDist.setCursor((SCREEN_WIDTH - textWidth) / 2, 10);
  displayDist.print(buf);

  displayDist.setTextSize(1);
  displayDist.setCursor((SCREEN_WIDTH - 12) / 2, 48);
  displayDist.print("cm");
  displayDist.display();
}

static void fillEllipse(int16_t cx, int16_t cy, int16_t rx, int16_t ry, uint16_t color) {
  const int32_t ry2 = (int32_t)ry * ry;
  for (int16_t y = -ry; y <= ry; ++y) {
    const float inner = 1.0f - (float)(y * y) / (float)ry2;
    if (inner < 0) {
      continue;
    }
    const int16_t xMax = (int16_t)(rx * sqrtf(inner) + 0.5f);
    displayGame.drawFastHLine(cx - xMax, cy + y, 2 * xMax + 1, color);
  }
}

static void drawBrow(int16_t cx, int16_t cy) {
  for (int16_t x = -16; x <= 16; ++x) {
    const int16_t y = cy - 22 + (x * x) / 64;
    displayGame.drawPixel(cx + x, y, SH110X_WHITE);
    displayGame.drawPixel(cx + x, y + 1, SH110X_WHITE);
  }
}

static void drawOpenEye(int16_t cx, int16_t cy) {
  fillEllipse(cx, cy, 20, 12, SH110X_WHITE);
  fillEllipse(cx, cy, 18, 10, SH110X_BLACK);
  fillEllipse(cx, cy, 17, 9, SH110X_WHITE);
  displayGame.fillCircle(cx, cy + 1, 8, SH110X_BLACK);
  displayGame.fillCircle(cx - 3, cy - 2, 2, SH110X_WHITE);
  displayGame.drawPixel(cx - 2, cy - 1, SH110X_WHITE);
  fillEllipse(cx, cy - 9, 18, 5, SH110X_BLACK);
}

static void drawClosedEye(int16_t cx, int16_t cy, int16_t dir) {
  for (int16_t x = -18; x <= 18; ++x) {
    const int16_t y = cy + (x * x) / 80;
    displayGame.drawPixel(cx + x, y, SH110X_WHITE);
    displayGame.drawPixel(cx + x, y + 1, SH110X_WHITE);
    displayGame.drawPixel(cx + x, y + 2, SH110X_WHITE);
  }
  displayGame.drawLine(cx + dir * 10, cy + 1, cx + dir * 16, cy - 4, SH110X_WHITE);
  displayGame.drawLine(cx + dir * 6, cy + 2, cx + dir * 12, cy - 5, SH110X_WHITE);
  displayGame.drawLine(cx + dir * 14, cy + 2, cx + dir * 18, cy - 2, SH110X_WHITE);
}

static void drawEyes(bool closed) {
  selectMuxChannel(CH_FACE);
  displayGame.clearDisplay();

  const int16_t leftX = 36;
  const int16_t rightX = 92;
  const int16_t eyeY = 34;

  drawBrow(leftX, eyeY);
  drawBrow(rightX, eyeY);
  if (closed) {
    drawClosedEye(leftX, eyeY, -1);
    drawClosedEye(rightX, eyeY, 1);
  } else {
    drawOpenEye(leftX, eyeY);
    drawOpenEye(rightX, eyeY);
  }

  displayGame.display();
  gEyesClosed = closed;
}

static void showEnemyDetected() {
  selectMuxChannel(CH_FACE);
  displayGame.clearDisplay();
  displayGame.setTextColor(SH110X_WHITE);
  displayGame.setTextSize(2);
  displayGame.setCursor((SCREEN_WIDTH - 5 * 12) / 2, 16);
  displayGame.print("ENEMY");
  displayGame.setCursor((SCREEN_WIDTH - 8 * 12) / 2, 36);
  displayGame.print("DETECTED");
  displayGame.display();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Sweep radar: enemy if closer than 20 cm");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(MUX_RESET_PIN, OUTPUT);
  digitalWrite(MUX_RESET_PIN, LOW);
  delay(10);
  digitalWrite(MUX_RESET_PIN, HIGH);
  delay(20);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  gServo.setPeriodHertz(50);
  gServo.attach(SERVO_PIN, 500, 2400);
  gAngle = SERVO_MIN;
  gServo.write(gAngle);
  delay(300);
  xTaskCreatePinnedToCore(servoTask, "servo", 2048, nullptr, 2, nullptr, 0);

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

  gDistReady = initOled(displayDist, CH_DIST);
  gGameReady = initOled(displayGame, CH_FACE);
  if (gGameReady) {
    drawEyes(false);
    gBlinkAtMs = millis();
  }
  gPingAtMs = millis();
}

void loop() {
  if (!gDistReady && !gGameReady) {
    delay(2000);
    return;
  }

  const uint32_t now = millis();

  if (gGameReady && !gEnemy) {
    if (!gEyesClosed && (now - gBlinkAtMs >= BLINK_EVERY_MS)) {
      drawEyes(true);
      gBlinkAtMs = now;
    } else if (gEyesClosed && (now - gBlinkAtMs >= BLINK_CLOSED_MS)) {
      drawEyes(false);
      gBlinkAtMs = now;
    }
  }

  if (now - gPingAtMs < PING_PERIOD_MS) {
    return;
  }
  gPingAtMs = now;

  const int cm = readDistanceCm();
  if (gDistReady) {
    showDistance(cm);
  }

  const int angle = gAngle;
  const bool close = (cm > 0) && (cm < ENEMY_RANGE_CM);
  if (close && !gEnemy) {
    gEnemy = true;
    if (gGameReady) {
      showEnemyDetected();
    }
    Serial.printf("ENEMY at %d deg  %d cm\n", angle, cm);
  } else if (!close && gEnemy) {
    gEnemy = false;
    if (gGameReady) {
      drawEyes(false);
    }
    gBlinkAtMs = now;
    Serial.println("watching");
  }

  if (cm < 0) {
    Serial.printf("ang %d  ---\n", angle);
  } else {
    Serial.printf("ang %d  %d cm\n", angle, cm);
  }
}
