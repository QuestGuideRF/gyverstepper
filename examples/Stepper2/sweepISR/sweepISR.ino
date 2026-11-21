#include "GyverStepper2.h"
GStepper2<STEPPER2WIRE> stepper(2048, 2, 5);
void setup() {
  Serial.begin(9600);
  initTimer();
  stepper.setMaxSpeed(100);
  stepper.setAcceleration(200);
  stepper.setTarget(300);
  setPeriod(stepper.getPeriod());
  startTimer();
}
ISR(TIMER1_COMPA_vect) {
  if (stepper.tickManual()) setPeriod(stepper.getPeriod());
  else stopTimer();
}
bool dir = 1;
void loop() {
  if (stepper.ready()) {
    dir = !dir;
    stepper.setTarget(dir * 300);
    setPeriod(stepper.getPeriod());
    startTimer();
  }
  static uint32_t tmr;
  if (millis() - tmr >= 30) {
    tmr = millis();
    Serial.println(stepper.pos);
  }
}