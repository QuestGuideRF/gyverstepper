#define GS_FAST_PROFILE 10
#include "GyverStepper2.h"
GStepper2<STEPPER2WIRE> stepper(2048, 2, 5);
uint32_t tar = 60000;
bool dir = 1;
void setup() {
  Serial.begin(9600);
  stepper.setMaxSpeed(30000);
  stepper.setAcceleration(30000);
  stepper.setTarget(tar);
}
void loop() {
  while (1) {
    stepper.tick();
    if (stepper.ready()) {
      dir = !dir;
      stepper.setTarget(dir * tar);
    }
    static uint32_t tmr;
    if (millis() - tmr >= 30) {
      tmr = millis();
      Serial.println(stepper.pos);
    }
  }
}