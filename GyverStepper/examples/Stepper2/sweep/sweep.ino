#include "GyverStepper2.h"
GStepper2<STEPPER2WIRE> stepper(2048, 2, 5);
void setup() {
  Serial.begin(9600);
  stepper.setMaxSpeed(100);
  stepper.setAcceleration(200);
  stepper.setTarget(300);
}
bool dir = 1;
void loop() {
  stepper.tick();
  if (stepper.ready()) {
    dir = !dir;
    stepper.setTarget(dir * 300);
  }
  static uint32_t tmr;
  if (millis() - tmr >= 30) {
    tmr = millis();
    Serial.println(stepper.pos);
  }
}