#include <Servo.h>
Servo stamp;

const int buttonPin = 5; //marked controller pins
const int servoPin  = 6;

const int potPin = A0;
const int readyLight = 9;

const int originPoint = 0; //servo step range values
const int stampMax    = 58;

// AUTO timings
const unsigned long downTime  = 450;  // time held at stamp position
const unsigned long upTime    = 250;  // time after returning to 0
const unsigned long debounceTime = 25;


const int potZeroDeadband = 2;  // reduce noise with potentiometer

enum State { READY, MANUAL, AUTO_DOWN, AUTO_UP, WAIT_RELEASE };
State state = READY;

unsigned long t0 = 0;

unsigned long lastChangeMs = 0; //button debouncing for correct states
bool lastRaw = HIGH;
bool stableButton = HIGH;

bool readButtonDebounced() { //button reading and wait time without delay disruption
  bool raw = digitalRead(buttonPin);

  if (raw != lastRaw) {
    lastRaw = raw;
    lastChangeMs = millis();
  }
  if (millis() - lastChangeMs >= debounceTime) {
    stableButton = raw;
  }
  return stableButton;
}

int readPotPos() { //potentiometer reading and timing 
  int pot = analogRead(potPin);
  int pos = map(pot, 0, 1023, 0, stampMax);
  if (pos < 0) pos = 0;
  if (pos > stampMax) pos = stampMax;

  if (pos <= potZeroDeadband) pos = 0; // deadband
  return pos;
}

void setup() {
  pinMode(buttonPin, INPUT_PULLUP); //internal resistor
  pinMode(readyLight, OUTPUT); //external resistor ~220-330

  stamp.attach(servoPin); 
  stamp.write(originPoint);
}

void loop() {
  const bool btn = readButtonDebounced(); // HIGH = released, LOW = pressed
  const int potPos = readPotPos();

  // LED ON when machine is not in use, required action either button or potentiometer
  digitalWrite(readyLight, (state == READY && potPos == 0) ? HIGH : LOW);

  switch (state) {

    case READY: {
      // if user moves the potentiometer away from 0, enter MANUAL and lock out button
      if (potPos != 0) {
        stamp.write(potPos);
        state = MANUAL;
        break;
      }

      // when potentiometer is at 0 allow AUTO stamping on button press
      if (btn == LOW) {
        stamp.write(stampMax);
        t0 = millis();
        state = AUTO_DOWN;
      } else {
        // ensure stamp is at origin for action
        stamp.write(originPoint);
      }
    } break;

    case MANUAL: {
      // while pot in use, button is not available for use
      // servo follows potentiometer
      stamp.write(potPos);

      // exit MANUAL only when potentiometer returns to 0, no button mix errors
      if (potPos == 0) {
        stamp.write(originPoint);
        state = READY;
      }
    } break;

    case AUTO_DOWN: {
      // ignore potentiometer while AUTO
      if (millis() - t0 >= downTime) {
        stamp.write(originPoint);
        t0 = millis();
        state = AUTO_UP;
      }
    } break;

    case AUTO_UP: {
      // ignore potentiometer while AUTO
      if (millis() - t0 >= upTime) {
        state = WAIT_RELEASE; // don’t allow retrigger while held
      }
    } break;

    case WAIT_RELEASE: {
      // still ignore potentiometer, require button release to allow action again
      if (btn == HIGH) {
        state = READY;
      }
    } break;
  }
}