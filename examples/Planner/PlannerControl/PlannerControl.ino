#include "GyverPlanner.h"
Stepper<STEPPER2WIRE> stepper1(2, 5);
Stepper<STEPPER2WIRE> stepper2(3, 6);
GPlanner<STEPPER2WIRE, 2> planner;
void setup() {
  Serial.begin(57600);
  planner.addStepper(0, stepper1);
  planner.addStepper(1, stepper2);
  planner.setAcceleration(200);
  planner.setMaxSpeed(200);
}
byte count = 0;
int32_t path[][2] = {
  {0, 0},
  {100, 150},
  {200, 200},
};
void loop() {
  planner.tick();
  if (planner.ready()) {
    planner.setTarget(path[count]);
    if (++count >= sizeof(path) / 8) count = 0;
  }
  if (Serial.available() > 0) {
    char incoming = Serial.read();
    switch (incoming) {
      case 's': planner.stop(); break;
      case 'b': planner.brake(); break;
      case 'r': planner.resume(); break;
      case 'p': planner.pause(); break;
    }
  }
  static uint32_t tmr;
  if (millis() - tmr >= 20) {
    tmr = millis();
    Serial.print(planner.getTarget(0));
    Serial.print(',');
    Serial.print(planner.getTarget(1));
    Serial.print(',');
    Serial.print(stepper1.pos);
    Serial.print(',');
    Serial.println(stepper2.pos);
  }
}