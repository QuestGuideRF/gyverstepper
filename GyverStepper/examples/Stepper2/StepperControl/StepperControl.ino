#include "GyverStepper2.h"
GStepper2<STEPPER2WIRE> stepper(2048, 2, 5);
void setup() {
  Serial.begin(9600);
  stepper.setAcceleration(200);
  stepper.setMaxSpeed(100);
  stepper.setTarget(0);
}
byte count = 0;
int16_t path[] = {0, 200, 100};
void loop() {
  stepper.tick();
  if (stepper.ready()) {
    stepper.setTarget(path[count]);
    if (++count >= sizeof(path) / 2) count = 0;
  }
  if (Serial.available() > 0) {
    char incoming = Serial.read();
    switch (incoming) {
      case 's': stepper.stop(); break;
      case 'b': stepper.brake(); break;
      case 'r': stepper.resume(); break;
      case 'p': stepper.pause(); break;
    }
  }
  static uint32_t tmr;
  if (millis() - tmr >= 20) {
    tmr = millis();
    Serial.print(stepper.getTarget());
    Serial.print(',');
    Serial.println(stepper.pos);
  }
}