# STPM3X
Library to interface with the Power Monitor Chips STPM34 or STPM32 by ST Microelectronics.\
As these chips are meant to work with 230V, be sure to know what you are doing! Keep safe! This library just helps you to interface with the chips and gives some basic stuff on top like calibration.

```C++
#include "STPM3X.h"
...
STPM stpm3x(RES_PIN, CS_PIN, SYN_PIN);

float active, reactive, apparent, fundamental;

void setup() {
  Serial.begin(115200);
  stpm3x.init();
  ...
}

void loop() {
  stpm3x.readPower(1, &active, &fundamental, &reactive, &apparent);
  Serial.printf("Active: %.2fW, Reactive: %.2fvar, Apparent: %.2fVA\n", active, reactive, apparent);
  ...
}
```