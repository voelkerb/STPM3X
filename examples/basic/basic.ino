/***************************************************
 Example file for using the bistableRelay library.
 
 License: Creative Common V1. 

 Benjamin Voelker, voelkerb@me.com
 Embedded Systems Engineer
 ****************************************************/

#include "STPM3X.h"

// Pins for STPM SPI Connection
const int STPM_CS = 5;
const int STPM_SYN = 14;
const int STPM_RES = 12;

// STPM Object
STPM stpm3x(STPM_RES, STPM_CS, STPM_SYN);


void setup() {
  Serial.begin(115200);

  bool success = stpm3x.init();
  if (!success) Serial.println("STPM Init Failed");
  // Optionally, set calibration factor vor voltage and current
  // stpm3x.setCalibration(1.01, 1.1);
  // stpm3xsetCurrentGain(1, sixteenX);
}

void loop() {
  // TOGGLE
  delay(1000);
  float activePower = stpm3x.readActivePower(1); // in W
  float reactivePower = stpm3x.readActivePower(1); // in VA
  float rmsCurrent = stpm3x.readRMSCurrent(1); // in mA
  float rmsVoltage = stpm3x.readRMSVoltage(1); // in mA
  // STPM34 with more than one channel?
  // float rmsVoltage2 = stpm3x.readRMSVoltage(2); // in mA
  Serial.printf("Active Power:\t%.2fW\n", activePower);
  Serial.printf("Reactive Power:\t%.2fvar\n", reactivePower);
  Serial.printf("RMS Current:\t%.2fmA\n", rmsCurrent);
  Serial.printf("RMS Voltage:\t%.2fV\n", rmsVoltage);
  Serial.println("________________");
  // Faster way and not rms but momentary voltage and current (sine wave sampling)
  // stpm3x.readAll(1, &voltage, &current, &activePower, &reactivePower);
}
