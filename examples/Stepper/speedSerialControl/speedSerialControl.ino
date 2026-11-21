#include <GyverStepper.h>
GStepper<STEPPER2WIRE> stepper(2048, 2, 5);
void setup() {
  Serial.begin(9600);
  stepper.setRunMode(KEEP_SPEED);
  stepper.setSpeedDeg(5);
}
void loop() {
  stepper.tick();
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'q') stepper.brake();
    if (ch == 'w') stepper.stop();
    if (ch == 'e') stepper.setSpeedDeg(5);
    if (ch == 'r') stepper.setSpeedDeg(100);
  }
}