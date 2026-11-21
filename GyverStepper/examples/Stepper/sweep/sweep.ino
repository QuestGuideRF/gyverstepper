#include <GyverStepper.h>
GStepper<STEPPER2WIRE> stepper(2048, 2, 5);
void setup() {
  Serial.begin(115200);
  stepper.setRunMode(FOLLOW_POS);
  stepper.setMaxSpeed(400);
  stepper.setAcceleration(500);
}
void loop() {
  if (!stepper.tick()) {
    static bool dir;
    dir = !dir;
    stepper.setTarget(dir ? -400 : 400);
  }
  static uint32_t tmr2;
  if (millis() - tmr2 > 20) {
    tmr2 = millis();
    Serial.println(stepper.getCurrent());
  }
}