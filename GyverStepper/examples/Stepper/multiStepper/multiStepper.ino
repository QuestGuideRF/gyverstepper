#include "GyverStepper.h"
GStepper<STEPPER2WIRE> stepper1(100, 2, 3);
GStepper<STEPPER2WIRE> stepper2(100, 4, 5);
GStepper<STEPPER2WIRE> stepper3(100, 6, 7, 8);
void setup() {
  stepper1.setRunMode(KEEP_SPEED);
  stepper1.setSpeed(300);
  stepper2.setRunMode(FOLLOW_POS);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(300);
  stepper3.setRunMode(FOLLOW_POS);
  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(300);
  stepper3.autoPower(true);
  stepper3.enable();
}
void loop() {
  stepper1.tick();
  if (!stepper2.tick()) {
    static bool dir;
    dir = !dir;
    stepper2.setTarget(dir ? -1000 : 1000);
  }
  stepper3.tick();
  static uint32_t tmr;
  if (millis() - tmr > 5000) {
    tmr = millis();
    stepper3.setTarget(random(0, 2000));
  }
}