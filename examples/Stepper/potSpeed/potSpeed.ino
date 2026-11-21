#include <GyverStepper.h>
GStepper<STEPPER2WIRE> stepper(2048, 2, 5);
void setup() {
  stepper.setRunMode(KEEP_SPEED);
  stepper.setSpeedDeg(50);
}
void loop() {
  stepper.tick();
  static uint32_t tmr2;
  if (millis() - tmr2 > 50) {
    tmr2 = millis();
    stepper.setSpeed(512 - analogRead(0));
  }
}