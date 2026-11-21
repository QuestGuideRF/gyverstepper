#include <GyverStepper.h>
GStepper<STEPPER4WIRE> stepper(2048, 5, 3, 4, 2);
void setup() {
  Serial.begin(115200);
  stepper.setRunMode(KEEP_SPEED);
  stepper.setSpeed(120);
  stepper.setSpeedDeg(80);
  stepper.setRunMode(FOLLOW_POS);
  stepper.setTarget(-2024);
  stepper.setTargetDeg(-360);
  stepper.setMaxSpeedDeg(400);
  stepper.setMaxSpeed(400);
  stepper.setAccelerationDeg(300);
  stepper.setAcceleration(300);
  stepper.autoPower(true);
  stepper.enable();
}
void loop() {
  if (!stepper.tick()) {
    static bool dir;
    dir = !dir;
    stepper.setTarget(dir ? -1024 : 1024);
  }
}