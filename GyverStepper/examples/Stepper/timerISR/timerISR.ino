#include <GyverStepper.h>
GStepper<STEPPER2WIRE> stepper(2048, 2, 5);
#include <GyverTimers.h>
void setup() {
  Serial.begin(115200);
  stepper.setRunMode(FOLLOW_POS);
  stepper.setMaxSpeed(400);
  stepper.setAcceleration(500);
  Timer2.setPeriod(stepper.getMinPeriod() / 2);
  Timer2.enableISR();
}
ISR(TIMER2_A) {
  stepper.tick();
}
void loop() {
  if (!stepper.tick()) {
    static bool dir;
    dir = !dir;
    stepper.setTarget(dir ? -400 : 400);
  }
  Serial.println(stepper.getCurrent());
  delay(100);
}