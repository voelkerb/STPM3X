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

## Calibration

You can calibrate the STPM3X output if e.g. your voltage dividers are designed differently or some component deviations prevent that you perfectly measure the real value. 
I.e. if your output is 200V but you measured it with laboratory equipment and it should be 230V, set a voltage calibration factor (`calV`) to `230.0/200.0=1.15`.

The basic calibration logic reads as:
```
float volt = convert2Volt(rawVolt) * calV;
float current = convert2Cur(rawCur) * calI;
```

1. Use a constant and defined load and connect it to your circuit.
2. Use a calibrated volt- and amperemeter to read the actual voltage and current applied to the STPM3X.
3. Read the output measured by the STPM3X
4. Use the `setCalibration(float calV, float calI)` function to add a custom multiplication factor. Offset correction is currently not supported. 

