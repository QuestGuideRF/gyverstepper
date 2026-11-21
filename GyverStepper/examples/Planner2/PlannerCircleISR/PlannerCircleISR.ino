const int pointAm = 30;
int radius = 100;
int32_t path[pointAm + 1 + 1][2];
#include "GyverPlanner2.h"
Stepper<STEPPER2WIRE> stepper1(2, 5);
Stepper<STEPPER2WIRE> stepper2(3, 6);
GPlanner2<STEPPER2WIRE, 2> planner;
void setup() {
  Serial.begin(115200);
  planner.addStepper(0, stepper1);
  planner.addStepper(1, stepper2);
  planner.setAcceleration(500);
  planner.setMaxSpeed(500);
  planner.setCurrent(path[0]);
  for (int i = 0; i <= pointAm; i++) {
    path[i + 1][0] = radius + radius * cos(TWO_PI * i / pointAm);
    path[i + 1][1] = radius + radius * sin(TWO_PI * i / pointAm);
  }
  planner.start();
  initTimer();
}
ISR(TIMER1_COMPA_vect) {
  if (planner.tickManual()) setPeriod(planner.getPeriod());
  else stopTimer();
}
int count = 0;
void loop() {
  if (planner.checkBuffer()) {
    startTimer();
    setPeriod(planner.getPeriod());
  }
  if (planner.available()) {
    planner.addTarget(path[count], 0);
    if (++count >= sizeof(path) / 8) count = 0;
  }
  static uint32_t tmr;
  if (millis() - tmr >= 20) {
    tmr = millis();
    Serial.print(stepper1.pos);
    Serial.print(',');
    Serial.println(stepper2.pos);
  }
}