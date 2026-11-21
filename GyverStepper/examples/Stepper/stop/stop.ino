#include <GyverStepper.h>
GStepper<STEPPER2WIRE> stepper(2048, 2, 5);
void setup() {
  Serial.begin(115200);
  stepper.setMaxSpeed(400);
  stepper.setAcceleration(300);
  stepper.setRunMode(FOLLOW_POS);
  stepper.setTarget(-2024);
}
void loop() {
  stepper.tick();
  static uint32_t tmr;
  static bool flag = false;
  if (!flag && millis() > 3000) {
    stepper.stop();
    flag = true;
  }
  static uint32_t tmr2;
  if (millis() - tmr2 > 20) {
    tmr2 = millis();
    Serial.print(stepper.getTarget());
    Serial.print(',');
    Serial.println(stepper.getCurrent());
  }
}