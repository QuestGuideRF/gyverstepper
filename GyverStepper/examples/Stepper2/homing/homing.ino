#define LIMSW_X 6
#include "GyverStepper2.h"
GStepper2<STEPPER2WIRE> stepper(2048, 2, 5);
void setup() {
  pinMode(LIMSW_X, INPUT_PULLUP);
}
void loop() {
}
void homing() {
  if (digitalRead(LIMSW_X)) {
    stepper.setSpeed(-10);
    while (digitalRead(LIMSW_X)) {
      stepper.tick();
    }
    stepper.brake();
  }
  stepper.reset();
}