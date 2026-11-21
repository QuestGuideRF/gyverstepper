#include <GyverStepper.h>
GStepper<STEPPER2WIRE> stepper(2048, 2, 5);
void setup() {
  Serial.begin(115200);
  stepper.setRunMode(FOLLOW_POS);
  stepper.setMaxSpeed(400);
  stepper.setAcceleration(500);
  stepper.autoPower(true);
}
void loop() {
  stepper.tick();
  static uint32_t tmr2;
  if (millis() - tmr2 > 20) {
    tmr2 = millis();
    static float val;
    val += (analogRead(0) - val) * 0.08;
    stepper.setTarget(val);
    Serial.print(stepper.getTarget());
    Serial.print(',');
    Serial.println(stepper.getCurrent());
  }
}