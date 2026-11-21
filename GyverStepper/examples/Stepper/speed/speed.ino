#include <GyverStepper.h>
GStepper<STEPPER2WIRE> stepper(2048, 2, 5);
void setup() {
  stepper.setRunMode(KEEP_SPEED);
  stepper.setSpeedDeg(50);
}
void loop() {
  stepper.tick();
}