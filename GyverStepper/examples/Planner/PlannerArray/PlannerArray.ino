int16_t path[][2] = {
  {10, 10},
  {100, 5},
  {200, 200},
  {150, 150},
  {0, 0},
  {0, 50},
};
int nodeAmount = sizeof(path) / 4;
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
}
int count = 0;
void loop() {
  planner.tick();
  if (planner.ready()) {
    if (count < nodeAmount) {
      planner.setTarget(path[count]);
      count++;
    }
  }
  static uint32_t tmr;
  if (millis() - tmr >= 20) {
    tmr = millis();
    Serial.print(stepper1.pos);
    Serial.print(',');
    Serial.println(stepper2.pos);
  }
}