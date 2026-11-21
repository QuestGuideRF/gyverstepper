#include "GyverPlanner.h"
Stepper<STEPPER2WIRE> stepper1(2, 5);
Stepper<STEPPER2WIRE> stepper2(3, 6);
GPlanner<STEPPER2WIRE, 2> planner;
void setup() {
  Serial.begin(115200);
  planner.addStepper(0, stepper1);
  planner.addStepper(1, stepper2);
  planner.setAcceleration(100);
  planner.setMaxSpeed(300);
  planner.reset();
  int target[] = {300, 200};
  planner.setTarget(target);
}
void loop() {
  planner.tick();
  if (planner.ready()) {
    int newTarget[] = {10, 50};
    planner.setTarget(newTarget);
  }
  static uint32_t tmr;
  if (millis() - tmr >= 20) {
    tmr = millis();
    Serial.print(stepper1.pos);
    Serial.print(',');
    Serial.println(stepper2.pos);
  }
}