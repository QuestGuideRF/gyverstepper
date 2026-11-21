int path[][2] = {
  {100, 250},
  {160, 30},
  {230, 250},
  {60, 100},
  {270, 100},
};
int nodeAmount = sizeof(path) / 4;
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
  planner.start();
}
int count = 0;
void loop() {
  planner.tick();
  if (planner.available()) {
    planner.addTarget(path[count], 0);
    if (++count >= sizeof(path) / 4) count = 0;
  }
  static uint32_t tmr;
  if (millis() - tmr >= 20) {
    tmr = millis();
    Serial.print(stepper1.pos);
    Serial.print(',');
    Serial.println(stepper2.pos);
  }
}