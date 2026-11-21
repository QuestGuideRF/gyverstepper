#include <GyverStepper.h>
GStepper<STEPPER2WIRE> stepper(2048, 2, 5);
void setup() {
  Serial.begin(115200);
  pinMode(A0, INPUT_PULLUP);
  stepper.enable();
  stepper.autoPower(true);
  stepper.setAcceleration(1200);
  stepper.setRunMode(KEEP_SPEED);
}
bool btnState = false;
void loop() {
  stepper.tick();
  if (!digitalRead(A0) && !btnState) {
    btnState = true;
    stepper.setSpeed(3000, SMOOTH);
  }
  if (digitalRead(A0) && btnState) {
    btnState = false;
    stepper.stop();
  }
}