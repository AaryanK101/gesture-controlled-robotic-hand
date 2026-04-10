#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/*
  Gesture-Controlled Robotic Hand
  --------------------------------
  This system reads finger bend data from five flex sensors mounted on a glove,
  classifies each finger as open or closed, and reproduces the detected posture
  on a servo-driven robotic hand.

  Two operating modes are supported:
  1. Gesture Copy Mode
     The robotic hand mirrors the glove in real time.
  2. Rock-Paper-Scissors Mode
     The glove is used as a gesture input device for an interactive game.

  Key implementation features:
  - Runtime calibration for open and closed finger positions
  - Smoothed analog sampling for improved measurement stability
  - Hysteresis-based state detection to reduce flicker near thresholds
  - OLED feedback for calibration, live status, and game screens
*/

const float VCC = 5.0;
const float R_FIXED = 22000.0;

/*
  Analog input pins connected to the flex sensor voltage dividers.
  Each pin corresponds to one finger on the glove.
*/
const int thumbPin  = A5;
const int indexPin  = A4;
const int middlePin = A3;
const int ringPin   = A2;
const int pinkyPin  = A1;

/*
  PWM output pins used to drive the five servo motors inside the robotic hand.
*/
const int thumbServoPin  = 3;
const int indexServoPin  = 5;
const int middleServoPin = 6;
const int ringServoPin   = 9;
const int pinkyServoPin  = 11;

/*
  Push button assignments:
  - BTN_RESET switches to Gesture Copy Mode and is also used during calibration
  - BTN_MODE switches to Rock-Paper-Scissors Mode
*/
const int BTN_RESET = 4;
const int BTN_MODE  = 2;
const int DEBOUNCE_DELAY = 50;

/*
  OLED display pin configuration.
*/
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_CLK       13
#define OLED_MOSI      10
#define OLED_RESET      7
#define OLED_DC         8
#define OLED_CS        12

/*
  Servo inversion flags compensate for differences in mechanical mounting
  so that logical open and closed states produce the correct physical motion.
*/
const bool thumbInverted  = true;
const bool indexInverted  = true;
const bool middleInverted = false;
const bool ringInverted   = false;
const bool pinkyInverted  = true;

/*
  Calibration values are captured at runtime.
  Each finger stores one reference for the open position and one for the closed position.
*/
float thumbOpen, thumbClosed;
float indexOpen, indexClosed;
float middleOpen, middleClosed;
float ringOpen, ringClosed;
float pinkyOpen, pinkyClosed;

/*
  Peripheral objects.
*/
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
                         OLED_MOSI, OLED_CLK, OLED_DC,
                         OLED_RESET, OLED_CS);

Servo thumbServo, indexServo, middleServo, ringServo, pinkyServo;

/*
  System state variables.
*/
bool modeCopying = true;

bool lastStableReset = false, lastStableMode = false;
unsigned long lastDebounceReset = 0, lastDebounceMode = 0;
int lastRawReset = LOW, lastRawMode = LOW;

bool thumbClosedState  = false;
bool indexClosedState  = false;
bool middleClosedState = false;
bool ringClosedState   = false;
bool pinkyClosedState  = false;

bool prevThumb  = false, prevIndex  = false, prevMiddle = false;
bool prevRing   = false, prevPinky  = false;

/*
  Game state machine for Rock-Paper-Scissors mode.
*/
enum GameState { IDLE, COUNTDOWN, READING, RESULT };
GameState gameState     = IDLE;
GameState prevGameState = IDLE;

int playerScore = 0;
int aiScore     = 0;
int roundNum    = 0;

/*
  Returns an estimate of free SRAM for debugging and memory monitoring.
*/
int getFreeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

/*
  Prints finger-state changes to the serial monitor.
  This is useful when tuning thresholds and verifying detection logic.
*/
void printFingerStates() {
  if (thumbClosedState  != prevThumb  ||
      indexClosedState  != prevIndex  ||
      middleClosedState != prevMiddle ||
      ringClosedState   != prevRing   ||
      pinkyClosedState  != prevPinky) {

    Serial.print(F("FINGERS | "));
    Serial.print(F("THB:")); Serial.print(thumbClosedState  ? F("C") : F("O"));
    Serial.print(F(" IDX:")); Serial.print(indexClosedState  ? F("C") : F("O"));
    Serial.print(F(" MID:")); Serial.print(middleClosedState ? F("C") : F("O"));
    Serial.print(F(" RNG:")); Serial.print(ringClosedState   ? F("C") : F("O"));
    Serial.print(F(" PNK:")); Serial.print(pinkyClosedState  ? F("C") : F("O"));
    Serial.println();

    prevThumb  = thumbClosedState;
    prevIndex  = indexClosedState;
    prevMiddle = middleClosedState;
    prevRing   = ringClosedState;
    prevPinky  = pinkyClosedState;
  }
}

/*
  Prints the current game state for serial debugging.
*/
void printGameState(GameState s) {
  Serial.print(F("GAMESTATE >> "));
  switch (s) {
    case IDLE:      Serial.println(F("IDLE"));      break;
    case COUNTDOWN: Serial.println(F("COUNTDOWN")); break;
    case READING:   Serial.println(F("READING"));   break;
    case RESULT:    Serial.println(F("RESULT"));    break;
  }
}

/*
  Updates the game state only when a transition occurs.
*/
void setGameState(GameState s) {
  if (s != gameState) {
    gameState = s;
    printGameState(s);
  }
}

/*
  Converts a logical finger state into a servo angle.
  A closed finger maps to one end-stop and an open finger maps to the other.
*/
int stateToAngle(bool closedState, bool inverted) {
  if (!inverted) return closedState ? 180 : 0;
  return closedState ? 0 : 180;
}

/*
  Sends the current logical finger states to the five servo motors.
*/
void writeServos(bool thumb, bool index, bool middle, bool ring, bool pinky) {
  thumbServo.write(stateToAngle(thumb, thumbInverted));
  indexServo.write(stateToAngle(index, indexInverted));
  middleServo.write(stateToAngle(middle, middleInverted));
  ringServo.write(stateToAngle(ring, ringInverted));
  pinkyServo.write(stateToAngle(pinky, pinkyInverted));
}

/*
  Predefined hand poses used in the system.
*/
void poseOpen()     { writeServos(false, false, false, false, false); }
void poseRock()     { writeServos(true,  true,  true,  true,  true);  }
void posePaper()    { poseOpen(); }
void poseScissors() { writeServos(true,  false, false, true,  true);  }

/*
  Reads one flex sensor multiple times, filters saturated readings,
  computes an average ADC value, and converts it to sensor resistance.

  The flex sensor forms part of a voltage divider with R_FIXED.
*/
float readFlexOhmsSmoothed(int pin) {
  long total = 0;
  int count  = 0;

  for (int i = 0; i < 15; i++) {
    int adc = analogRead(pin);
    float v = adc * VCC / 1023.0;

    if (v < VCC - 0.01) {
      total += adc;
      count++;
    }

    delay(2);
  }

  if (count == 0) return -1;

  float adcAvg = total / (float)count;
  float v = adcAvg * VCC / 1023.0;

  return R_FIXED * (v / (VCC - v));
}

/*
  Applies hysteresis to determine whether a finger should be considered open or closed.

  closeFactor defines the threshold required to enter the closed state.
  openFactor defines the threshold required to return to the open state.

  Using separate thresholds helps prevent rapid state toggling near the boundary.
*/
bool updateClosedState(float r, float rOpen, float rClosed, bool cur,
                       float closeFactor, float openFactor) {
  if (r < 0) return cur;

  float cThr = rOpen + closeFactor * (rClosed - rOpen);
  float oThr = rOpen + openFactor  * (rClosed - rOpen);

  if (rClosed > rOpen) {
    if (!cur && r >= cThr) return true;
    if ( cur && r <= oThr) return false;
  } else {
    if (!cur && r <= cThr) return true;
    if ( cur && r >= oThr) return false;
  }

  return cur;
}

/*
  Reads all five flex sensors and updates the binary finger states.
  Thresholds are tuned independently for each finger.
*/
void readFingers() {
  float rT = readFlexOhmsSmoothed(thumbPin);
  float rI = readFlexOhmsSmoothed(indexPin);
  float rM = readFlexOhmsSmoothed(middlePin);
  float rR = readFlexOhmsSmoothed(ringPin);
  float rP = readFlexOhmsSmoothed(pinkyPin);

  thumbClosedState  = updateClosedState(rT, thumbOpen,  thumbClosed,  thumbClosedState,  0.60, 0.25);
  indexClosedState  = updateClosedState(rI, indexOpen,  indexClosed,  indexClosedState,  0.55, 0.35);
  middleClosedState = updateClosedState(rM, middleOpen, middleClosed, middleClosedState, 1.00, 0.80);
  ringClosedState   = updateClosedState(rR, ringOpen,   ringClosed,   ringClosedState,   0.60, 0.40);
  pinkyClosedState  = updateClosedState(rP, pinkyOpen,  pinkyClosed,  pinkyClosedState,  0.60, 0.25);

  printFingerStates();
}

/*
  Detects one of the supported gestures from the current finger states.

  Gesture encoding:
  0 = Rock
  1 = Paper
  2 = Scissors
 -1 = No valid gesture detected
*/
int detectGesture() {
  bool isRock     =  thumbClosedState &&  indexClosedState &&
                     middleClosedState &&  ringClosedState &&  pinkyClosedState;

  bool isScissors = !indexClosedState && !middleClosedState &&
                     thumbClosedState  &&  ringClosedState  &&  pinkyClosedState;

  bool isPaper    = !indexClosedState && !middleClosedState &&
                    !ringClosedState  && !pinkyClosedState;

  if (isRock)     return 0;
  if (isPaper)    return 1;
  if (isScissors) return 2;
  return -1;
}

/*
  Converts a gesture code into a readable label.
*/
const char* gestureName(int g) {
  if (g == 0) return "ROCK";
  if (g == 1) return "PAPER";
  if (g == 2) return "SCISSORS";
  return "???";
}

/*
  Samples one finger repeatedly during calibration and returns the mean resistance.
*/
float sampleFinger(int pin) {
  float total = 0;
  int count = 0;

  for (int i = 0; i < 20; i++) {
    float r = readFlexOhmsSmoothed(pin);
    if (r > 0) {
      total += r;
      count++;
    }
    delay(50);
  }

  return count > 0 ? total / count : 0;
}

/*
  Waits for the calibration confirmation button.
*/
void waitForButton() {
  while (digitalRead(BTN_RESET) == LOW) {
    delay(10);
  }
  delay(300);
}

/*
  Runs the startup calibration sequence.

  The user is asked to:
  1. hold all fingers open
  2. hold all fingers closed

  Reference resistances are then captured for each finger and used
  in all later classification steps.
*/
void runCalibration() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  centreText(F("CALIBRATION"), 0, 1);
  display.drawFastHLine(0, 11, SCREEN_WIDTH, SSD1306_WHITE);
  centreText(F("Open all fingers"), 18, 1);
  centreText(F("fully & hold"), 30, 1);
  display.drawFastHLine(0, 44, SCREEN_WIDTH, SSD1306_WHITE);
  centreText(F("Press BTN 4"), 48, 1);
  centreText(F("when ready"), 56, 1);
  display.display();

  Serial.println(F("CALIBRATION: waiting for OPEN position..."));
  waitForButton();

  display.clearDisplay();
  centreText(F("CALIBRATION"), 0, 1);
  display.drawFastHLine(0, 11, SCREEN_WIDTH, SSD1306_WHITE);
  centreText(F("Sampling OPEN..."), 24, 1);
  centreText(F("Hold still!"), 36, 1);
  display.display();

  thumbOpen  = sampleFinger(thumbPin);
  indexOpen  = sampleFinger(indexPin);
  middleOpen = sampleFinger(middlePin);
  ringOpen   = sampleFinger(ringPin);
  pinkyOpen  = sampleFinger(pinkyPin);

  Serial.print(F("OPEN  | T:")); Serial.print(thumbOpen);
  Serial.print(F(" I:"));        Serial.print(indexOpen);
  Serial.print(F(" M:"));        Serial.print(middleOpen);
  Serial.print(F(" R:"));        Serial.print(ringOpen);
  Serial.print(F(" P:"));        Serial.println(pinkyOpen);

  display.clearDisplay();
  centreText(F("CALIBRATION"), 0, 1);
  display.drawFastHLine(0, 11, SCREEN_WIDTH, SSD1306_WHITE);
  centreText(F("OPEN done!"), 24, 1);
  display.display();
  delay(800);

  display.clearDisplay();
  centreText(F("CALIBRATION"), 0, 1);
  display.drawFastHLine(0, 11, SCREEN_WIDTH, SSD1306_WHITE);
  centreText(F("Close all fingers"), 18, 1);
  centreText(F("fully & hold"), 30, 1);
  display.drawFastHLine(0, 44, SCREEN_WIDTH, SSD1306_WHITE);
  centreText(F("Press BTN 4"), 48, 1);
  centreText(F("when ready"), 56, 1);
  display.display();

  Serial.println(F("CALIBRATION: waiting for CLOSED position..."));
  waitForButton();

  display.clearDisplay();
  centreText(F("CALIBRATION"), 0, 1);
  display.drawFastHLine(0, 11, SCREEN_WIDTH, SSD1306_WHITE);
  centreText(F("Sampling CLOSED..."), 24, 1);
  centreText(F("Hold still!"), 36, 1);
  display.display();

  thumbClosed  = sampleFinger(thumbPin);
  indexClosed  = sampleFinger(indexPin);
  middleClosed = sampleFinger(middlePin);
  ringClosed   = sampleFinger(ringPin);
  pinkyClosed  = sampleFinger(pinkyPin);

  Serial.print(F("CLOSED | T:")); Serial.print(thumbClosed);
  Serial.print(F(" I:"));         Serial.print(indexClosed);
  Serial.print(F(" M:"));         Serial.print(middleClosed);
  Serial.print(F(" R:"));         Serial.print(ringClosed);
  Serial.print(F(" P:"));         Serial.println(pinkyClosed);

  display.clearDisplay();
  centreText(F("CALIBRATION"), 0, 1);
  display.drawFastHLine(0, 11, SCREEN_WIDTH, SSD1306_WHITE);
  centreText(F("CLOSED done!"), 24, 1);
  display.display();
  delay(800);

  display.clearDisplay();
  centreText(F("CALIBRATION"), 0, 1);
  display.drawFastHLine(0, 11, SCREEN_WIDTH, SSD1306_WHITE);
  centreText(F("Stabilising..."), 20, 1);
  centreText(F("Hold still!"), 34, 1);
  display.display();

  Serial.println(F("CALIBRATION complete! Stabilising..."));

  for (int i = 0; i < 20; i++) {
    readFingers();
    delay(50);
  }

  display.clearDisplay();
  centreText(F("CALIBRATION"), 0, 1);
  display.drawFastHLine(0, 11, SCREEN_WIDTH, SSD1306_WHITE);
  centreText(F("Complete!"), 20, 2);
  centreText(F("Starting..."), 44, 1);
  display.display();
  delay(600);

  lastStableReset = false;
  lastStableMode = false;
  lastRawReset = LOW;
  lastRawMode = LOW;
  lastDebounceReset = 0;
  lastDebounceMode = 0;
}

/*
  Draws centered text using the OLED display.
*/
void centreText(const __FlashStringHelper* str, int y, int sz) {
  display.setTextSize(sz);
  display.setTextColor(SSD1306_WHITE);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(str, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, y);
  display.print(str);
}

void centreText(const char* str, int y, int sz) {
  display.setTextSize(sz);
  display.setTextColor(SSD1306_WHITE);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(str, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, y);
  display.print(str);
}

/*
  Draws the shared scoreboard header used in game screens.
*/
void drawRPSHeader() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(2, 1);
  display.print(F("YOU:"));
  display.print(playerScore);

  char rbuf[6];
  snprintf(rbuf, sizeof(rbuf), "R%d", roundNum);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(rbuf, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 1);
  display.print(rbuf);

  char abuf[8];
  snprintf(abuf, sizeof(abuf), "CPU:%d", aiScore);
  display.getTextBounds(abuf, 0, 0, &x1, &y1, &w, &h);
  display.setCursor(SCREEN_WIDTH - w - 2, 1);
  display.print(abuf);

  display.drawFastHLine(0, 11, SCREEN_WIDTH, SSD1306_WHITE);
}

/*
  Gesture Copy Mode screen.
  Each finger is shown as OPEN or CLOSED.
*/
void screenGestureCopy() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(2, 1);
  display.print(F("GESTURE COPY"));
  display.setCursor(86, 1);
  display.print(F("[4]=COPY"));
  display.drawFastHLine(0, 11, SCREEN_WIDTH, SSD1306_WHITE);

  const char* labels[5] = { "THB", "IDX", "MID", "RNG", "PNK" };
  bool states[5] = {
    thumbClosedState, indexClosedState, middleClosedState,
    ringClosedState, pinkyClosedState
  };

  for (int i = 0; i < 5; i++) {
    int y = 14 + i * 10;

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(2, y);
    display.print(labels[i]);

    int barX = 30;
    int barW = 88;
    int barH = 8;

    if (states[i]) {
      display.fillRect(barX, y, barW, barH, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(barX + 28, y);
      display.print(F("CLOSED"));
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.drawRect(barX, y, barW, barH, SSD1306_WHITE);
      display.setCursor(barX + 32, y);
      display.print(F("OPEN"));
    }
  }

  display.display();
}

/*
  Idle screen for Rock-Paper-Scissors mode.
*/
void screenIdle() {
  display.clearDisplay();
  centreText(F("RPS GAME"), 4, 2);
  display.drawFastHLine(10, 22, 108, SSD1306_WHITE);
  centreText(F("Form a gesture"), 28, 1);
  centreText(F("to start!"), 40, 1);
  display.drawFastHLine(10, 51, 108, SSD1306_WHITE);

  char buf[24];
  snprintf(buf, sizeof(buf), "YOU %d  -  %d CPU", playerScore, aiScore);
  centreText(buf, 54, 1);

  display.display();
}

/*
  Countdown screen shown before gesture capture.
*/
void screenCountdown(int n) {
  display.clearDisplay();
  drawRPSHeader();

  char buf[4];
  snprintf(buf, sizeof(buf), "%d", n);

  display.setTextSize(5);
  display.setTextColor(SSD1306_WHITE);

  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 14);
  display.print(buf);

  display.display();
}

/*
  Prompt shown when the player should reveal a gesture.
*/
void screenShoot() {
  display.clearDisplay();
  drawRPSHeader();
  display.drawRect(14, 14, 100, 28, SSD1306_WHITE);
  centreText(F("SHOOT!"), 20, 2);
  centreText(F("Show your hand!"), 48, 1);
  display.display();
}

/*
  Screen displayed during gesture sampling.
*/
void screenReading() {
  display.clearDisplay();
  drawRPSHeader();
  centreText(F("Reading..."), 28, 1);
  display.display();
}

/*
  Final result screen for a completed round.
*/
void screenResult(int player, int ai, int outcome) {
  display.clearDisplay();
  drawRPSHeader();
  display.drawFastVLine(63, 11, 36, SSD1306_WHITE);

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(4, 14);  display.print(F("YOU"));
  display.setCursor(68, 14); display.print(F("CPU"));

  display.drawFastHLine(0, 23, SCREEN_WIDTH, SSD1306_WHITE);

  display.setCursor(4, 27);  display.print(gestureName(player));
  display.setCursor(66, 27); display.print(gestureName(ai));

  display.drawFastHLine(0, 47, SCREEN_WIDTH, SSD1306_WHITE);

  if (outcome == 1) {
    display.fillRect(0, 48, SCREEN_WIDTH, 16, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    centreText(F("** YOU WIN! **"), 52, 1);
    display.setTextColor(SSD1306_WHITE);
  } else if (outcome == -1) {
    centreText(F("CPU WINS"), 52, 1);
  } else {
    centreText(F("~ DRAW ~"), 52, 1);
  }

  display.display();
}

/*
  Reads and debounces both push buttons.

  BTN_RESET:
    Switches to Gesture Copy Mode

  BTN_MODE:
    Switches to Rock-Paper-Scissors Mode and resets game scores
*/
void checkButtons() {
  unsigned long now = millis();

  int rawReset = digitalRead(BTN_RESET);
  if (rawReset != lastRawReset) lastDebounceReset = now;

  if ((now - lastDebounceReset) > DEBOUNCE_DELAY &&
      (rawReset == HIGH) != lastStableReset) {
    lastStableReset = (rawReset == HIGH);

    if (lastStableReset) {
      modeCopying = true;
      Serial.println(F("BTN >> Switching to GESTURE COPY"));

      poseOpen();
      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE);
      centreText(F("GESTURE COPY"), 24, 1);
      centreText(F("MODE"), 38, 1);
      display.display();
      delay(600);
    }
  }
  lastRawReset = rawReset;

  int rawMode = digitalRead(BTN_MODE);
  if (rawMode != lastRawMode) lastDebounceMode = now;

  if ((now - lastDebounceMode) > DEBOUNCE_DELAY &&
      (rawMode == HIGH) != lastStableMode) {
    lastStableMode = (rawMode == HIGH);

    if (lastStableMode) {
      modeCopying = false;
      Serial.println(F("BTN >> Switching to RPS GAME"));

      setGameState(IDLE);
      playerScore = 0;
      aiScore = 0;
      roundNum = 0;

      poseOpen();
      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE);
      centreText(F("RPS GAME"), 24, 1);
      centreText(F("MODE"), 38, 1);
      display.display();
      delay(600);
    }
  }
  lastRawMode = rawMode;
}

/*
  System initialization.
*/
void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(A0));

  pinMode(BTN_RESET, INPUT);
  pinMode(BTN_MODE, INPUT);

  if (!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("OLED init failed"));
    while (1);
  }

  display.clearDisplay();
  centreText(F("RPS GLOVE"), 10, 2);
  centreText(F("Get ready!"), 36, 1);
  display.display();
  delay(1800);

  thumbServo.attach(thumbServoPin);
  indexServo.attach(indexServoPin);
  middleServo.attach(middleServoPin);
  ringServo.attach(ringServoPin);
  pinkyServo.attach(pinkyServoPin);

  poseOpen();
  runCalibration();

  Serial.println(F("========================="));
  Serial.println(F("  RPS GLOVE READY"));
  Serial.println(F("  Mode: GESTURE COPY"));
  Serial.print(F("  Free RAM: "));
  Serial.println(getFreeRam());
  Serial.println(F("========================="));

  printGameState(gameState);
}

/*
  Main program loop.

  In Gesture Copy Mode:
    - finger states are read continuously
    - the robotic hand mirrors the glove in real time

  In Rock-Paper-Scissors Mode:
    - the system waits for a valid gesture
    - performs a countdown
    - samples multiple readings to vote on the player's gesture
    - generates an AI move
    - updates the score and result screen
*/
void loop() {
  checkButtons();

  if (modeCopying) {
    readFingers();
    writeServos(thumbClosedState, indexClosedState,
                middleClosedState, ringClosedState, pinkyClosedState);
    screenGestureCopy();
    delay(100);
    return;
  }

  if (gameState == IDLE) {
    poseOpen();
    screenIdle();
    delay(300);

    readFingers();
    int g = detectGesture();

    if (g >= 0) {
      roundNum++;
      Serial.print(F("Gesture detected to start round: "));
      Serial.println(gestureName(g));
      setGameState(COUNTDOWN);
    }
    return;
  }

  if (gameState == COUNTDOWN) {
    screenCountdown(3); delay(900);
    screenCountdown(2); delay(900);
    screenCountdown(1); delay(900);
    screenShoot();      delay(700);
    setGameState(READING);
    return;
  }

  if (gameState == READING) {
    screenReading();

    int votes[3] = {0, 0, 0};

    for (int i = 0; i < 10; i++) {
      readFingers();
      int g = detectGesture();
      if (g >= 0) votes[g]++;
      delay(100);
    }

    int playerGesture = -1;
    int best = 0;

    for (int i = 0; i < 3; i++) {
      if (votes[i] > best) {
        best = votes[i];
        playerGesture = i;
      }
    }

    int aiGesture = random(0, 3);

    if      (aiGesture == 0) poseRock();
    else if (aiGesture == 1) posePaper();
    else                     poseScissors();

    int outcome = 0;

    if (playerGesture < 0) {
      outcome = -1;
      aiScore++;
      Serial.println(F("No gesture detected - CPU wins round"));
    } else if (playerGesture == aiGesture) {
      outcome = 0;
    } else if ((playerGesture == 0 && aiGesture == 2) ||
               (playerGesture == 2 && aiGesture == 1) ||
               (playerGesture == 1 && aiGesture == 0)) {
      outcome = 1;
      playerScore++;
    } else {
      outcome = -1;
      aiScore++;
    }

    Serial.print(F("R")); Serial.print(roundNum);
    Serial.print(F(" | YOU: ")); Serial.print(gestureName(playerGesture));
    Serial.print(F(" | CPU: ")); Serial.print(gestureName(aiGesture));
    Serial.print(F(" | "));

    if (outcome == 1)       Serial.println(F("YOU WIN"));
    else if (outcome == -1) Serial.println(F("CPU WINS"));
    else                    Serial.println(F("DRAW"));

    Serial.print(F("Score | YOU: "));
    Serial.print(playerScore);
    Serial.print(F(" CPU: "));
    Serial.println(aiScore);

    screenResult(playerGesture, aiGesture, outcome);
    delay(3200);
    setGameState(IDLE);
    return;
  }
}
