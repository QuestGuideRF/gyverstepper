#include <GyverStepper.h>
GStepper<STEPPER2WIRE> stepper(2048, 2, 5);
void setup() {
  pinMode(12, INPUT_PULLUP);
  stepper.setRunMode(KEEP_SPEED);
  stepper.setSpeedDeg(-10);
  while(digitalRead(12)) {
    stepper.tick();
  }
  stepper.reset();
  stepper.setRunMode(FOLLOW_POS);
}
void loop() {
  static bool dir;
  if (!stepper.tick()) {
    dir = !dir;
    stepper.setTarget(dir ? 20 : 300);
  }
}