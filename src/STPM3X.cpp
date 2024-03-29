/***************************************************
 Library for getting energy-data out of a STPM3X.
 See example file to get an idea of how to use this
 class.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#include "STPM3X.h"
#include <SPI.h>



// Info:|31|30|29|28|27|26|25|24|23|22|21|20|19|18|17|16|15|14|13|12|11|10| 9| 8| 7| 6| 5| 4| 3| 2| 1| 0| 
// Info:| 0| 0| 0| 0| 1| 1| 1| 1| 0| 0| 1| 0| 0| 1| 1| 1| 0| 0| 0| 0| 0| 0| 1| 1| 0| 0| 1| 0| 0| 1| 1| 1| 


const char * energyNames[] = {"Active", "Fundamental", "Reactive", "Apparent"};

//SPISettings spiSettings(12000000, MSBFIRST, SPI_MODE3);
SPISettings spiSettings(12000000, MSBFIRST, SPI_MODE3);

DSP_CR100bits_t DSP_CR100bits;
DSP_CR101bits_t DSP_CR101bits;
DSP_CR200bits_t DSP_CR200bits;
DSP_CR201bits_t DSP_CR201bits;
DSP_CR400bits_t DSP_CR400bits;
US1_REG100bits_t US1_REG100bits;

DFE_CR101bits_t DFE_CR101bits;
DFE_CR201bits_t DFE_CR201bits;
DSP_CR301bits_t DSP_CR301bits;
DSP_CR500bits_t DSP_CR500bits;

DSP_SR100bits_t DSP_SR100bits;
DSP_SR101bits_t DSP_SR101bits;

STPM::STPM(int resetPin, int csPin, int synPin, int fnet) {
  RESET_PIN = resetPin;
  CS_PIN = csPin;
  SYN_PIN = synPin;
  _autoLatch = false;
  _crcEnabled = true;
  netFreq = fnet;
  for (uint8_t i = 0; i < 3; i++) {
    _calibration[i][0] = 1.0;
    _calibration[i][1] = 1.0;
  }
  totalEnergy = {0.0, 0.0, 0.0, 0.0, 0};
  ph1Energy = {0.0, 0.0, 0.0, 0.0, 0};
  ph2Energy = {0.0, 0.0, 0.0, 0.0, 0};
  energies[0] = &totalEnergy;
  energies[1] = &ph1Energy;
  energies[2] = &ph2Energy;
  energiesHp[0] = &totalEnergyHp;
  energiesHp[1] = &ph1EnergyHp;
  energiesHp[2] = &ph2EnergyHp;
  _logFunc = NULL;
}

STPM::STPM(int resetPin, int csPin) {
  STPM(resetPin, csPin, -1, 50);
}

bool STPM::init() {
  pinMode(CS_PIN, OUTPUT);
  if (SYN_PIN != -1) pinMode(SYN_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(CS_PIN, LOW);
  digitalWrite(RESET_PIN, LOW);
  delay(35);
  digitalWrite(RESET_PIN, HIGH);
  delay(35);
  digitalWrite(CS_PIN, HIGH);
  delay(2);
  // Init sequence by togling 3 times syn pin
  if (SYN_PIN != -1) {
    for (size_t i = 0; i < 3; i++) {
      digitalWrite(SYN_PIN, LOW);
      delay(2);
      digitalWrite(SYN_PIN, HIGH);
      delay(2);
    }
  }
  delay(2);
  digitalWrite(CS_PIN, LOW);
  delay(5);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();
  //SPI.beginTransaction(spiSettings);
  bool success = Init_STPM34();
  #ifdef DEBUG_DEEP
	Serial.println(F("End Init"));
  #endif
  return success;
}

bool STPM::Init_STPM34() {
  #ifdef DEBUG_DEEP
	Serial.println(F("Start Init_STPM34"));
  #endif
  uint8_t readAdd, writeAdd, dataLSB, dataMSB;

  //set Voltage Reference
  #ifdef DEBUG_DEEP
  Serial.println(F("Info:Set DSP Control Register 1 LSW"));
  #endif
  DSP_CR100bits.ENVREF1 = 1;       //enable internal Vref1 bit for CH0;
  DSP_CR100bits.TC1 = 0x02;         //set temperature compensation for CH0; Vref1=1.18v default
  readAdd = 0x00;
  writeAdd = 0x00;
  dataLSB = DSP_CR100bits.LSB;
  dataMSB = DSP_CR100bits.MSB;
  sendFrameCRC(0x00, 0x00, dataLSB, dataMSB); //write to  CR1 register

  #ifdef DEBUG_DEEP
  Serial.println(F("Info:Set DSP Control Register 1 MSW"));
  #endif
  
  DSP_CR101bits.BHPFV1 = 0;        //HPF enable voltage;DC cancellation
  DSP_CR101bits.BHPFC1 = 0;        //HPF enable current;DC cancellation
  DSP_CR101bits.BLPFV1 = 1;        //LPF wideband voltage;set up fundamental mode
  DSP_CR101bits.BLPFC1 = 1;        //LPF wideband current;set up fundamental mode
  DSP_CR101bits.LPW1 = 0x04;        //LED Division factor
  readAdd = 0x00;
  writeAdd = 0x01;
  dataLSB = DSP_CR101bits.LSB;
  dataMSB = DSP_CR101bits.MSB;
  sendFrameCRC(readAdd, writeAdd, dataLSB, dataMSB); //write to  CR1 register
  
 
  #ifdef DEBUG_DEEP
  Serial.println(F("Info:Set DSP Control Register 2 LSW"));
  #endif
  DSP_CR200bits.ENVREF2 = 1;       //enable internal Vref1 bit for CH1;
  DSP_CR200bits.TC2 = 0x02;        //set temperature compensation for CH1;  Vref2=1.18v default
  readAdd = 0x01;
  writeAdd = 0x02;
  dataLSB = DSP_CR200bits.LSB;
  dataMSB = DSP_CR200bits.MSB;
  sendFrameCRC(readAdd, writeAdd, dataLSB, dataMSB); //write to  CR2 register

  #ifdef DEBUG_DEEP
  Serial.println(F("Info:Set DSP Control Register 2 MSW"));
  #endif
  DSP_CR201bits.BHPFV2 = 0;//1;//0;        //HPF enable voltage;DC cancellation
  DSP_CR201bits.BHPFC2 = 0;//1;//0;        //HPF enable current;DC cancellation
  DSP_CR201bits.BLPFV2 = 1;        //LPF bypassed -  wideband voltage;set up fundamental mode
  DSP_CR201bits.BLPFC2 = 1;        //LPF bypassed -  wideband current;set up fundamental mode
  DSP_CR201bits.LPW2 = 0x04;        //LED Division factor
  readAdd = 0x02;
  writeAdd = 0x03;
  dataLSB = DSP_CR201bits.LSB;
  dataMSB = DSP_CR201bits.MSB;
  sendFrameCRC(readAdd, writeAdd, dataLSB, dataMSB);


  // Set current gain: 0x00 = 2, 0x01 = 4, 0x02 = 8, 0x03 = 16
  setCurrentGain(1, sixteenX);
  setCurrentGain(2, sixteenX);


  // Read gain, use gain to test if stpm is up and running
  readFrame(0x18, readBuffer);
  bool success = checkGain(1, readBuffer);
  
  #ifdef DEBUG_DEEP
  Serial.println(F("Info:GAIN (Bit 26/27):"));
  printRegister(readBuffer, "GainC1:");
  Serial.println(F("Info:GAIN (Bit 26/27):"));
  Serial.print(F("Info:SetGain1: "));
  Serial.println(success ? "Success" : "Fail");
  #endif

  // Read gain, use gain to test if stpm is up and running
  readFrame(0x1A, readBuffer);
  success = success && checkGain(2, readBuffer);

  #ifdef DEBUG_DEEP
  printRegister(readBuffer, "GainC2:");
  Serial.print(F("Info:SetGain2: "));
  Serial.println(success ? "Success" : "Fail");
  #endif

  #ifdef DEBUG_DEEP
  Serial.println(F("Info:LPW: (Bit 24-27)"));
  readFrame(0x0, readBuffer);
  printRegister(readBuffer, "LPW1:");
  readFrame(0x01, readBuffer);
  printRegister(readBuffer, "LPW2:");
  #endif
  // TODO: change back
  autoLatch(false);
  // autoLatch(true);
  CRC(false);

  #ifdef DEBUG_DEEP
	Serial.println(F("End Init_STPM34"));
  #endif
  return success;
}

void STPM::setCalibration(float calV, float calI) {

  for (uint8_t i = 0; i < 3; i++) {
    _calibration[i][0] = calV;
    _calibration[i][1] = calI;
  }
}

void STPM::CRC(bool enabled) {
  if (_crcEnabled == enabled) return;
  // Disable CRC
  if (!enabled) {
    #ifdef DEBUG_DEEP
    Serial.println(F("Info:Disable CRC"));
    #endif
    US1_REG100bits.CRC_EN=0;         //disable CRC polynomial
    sendFrameCRC(0x24,0x24,US1_REG100bits.LSB,US1_REG100bits.MSB);
  // Enable CRC
  } else {
    #ifdef DEBUG_DEEP
    Serial.println(F("Info:Enable CRC"));
    #endif
    US1_REG100bits.CRC_EN=1;         //disable CRC polynomial
    sendFrame(0x24,0x24,US1_REG100bits.LSB,US1_REG100bits.MSB);
  }
  _crcEnabled = enabled;
}


void STPM::autoLatch(bool enabled) {
  #ifdef DEBUG_DEEP
  Serial.println(F("Info:Set DSP Control Register 3 LSW"));
  if (enabled) {
    Serial.println(F("Info:Automatic latching"));
  } else {
    Serial.println(F("Info:Manual latching"));
  }
  #endif
  _autoLatch = enabled;
  if (_autoLatch) {
    DSP_CR301bits.SWAuto_Latch = 1;      // Automatic measurement register latch at 7.8125 kHz
    DSP_CR301bits.SW_Latch1 = 0;
    DSP_CR301bits.SW_Latch2 = 0;
  } else {
    DSP_CR301bits.SWAuto_Latch = 0;      // Automatic measurement register latch at 7.8125 kHz
    DSP_CR301bits.SW_Latch1 = 1;
    DSP_CR301bits.SW_Latch2 = 1;
  }
  if (_crcEnabled) sendFrameCRC(0x05, 0x05, DSP_CR301bits.LSB, DSP_CR301bits.MSB);
  else sendFrame(0x05, 0x05, DSP_CR301bits.LSB, DSP_CR301bits.MSB);
}


void STPM::readAll(uint8_t channel, float *voltage, float *current, float* active, float* reactive) {// Set current gain: 0x00 = 2, 0x01 = 4, 0x02 = 8, 0x03 = 16
  if (channel == 1) {
    address = PH1_Active_Power_Address;
  } else if (channel == 2) {
    address = PH2_Active_Power_Address;
  } else {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return;
  }
  if (!_autoLatch) latch();

  sendFrame(address, 0xff, 0xff, 0xff);
  SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  *active = calcPower(buffer0to28(readBuffer))*_calibration[channel][_V]*_calibration[channel][_I];
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  // We skip fundamental power here
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  *reactive = calcPower(buffer0to28(readBuffer))*_calibration[channel][_V]*_calibration[channel][_I];
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  // *apparent = calcPower(buffer0to28(readBuffer));
  address = V1_Data_Address;
  if (channel == 2) address = V2_Data_Address;
  sendFrame(address, 0xff, 0xff, 0xff);
  SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);
  //delayMicroseconds(1000);
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  *voltage = calcVolt(buffer0to32(readBuffer))*_calibration[channel][_V];
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  *current = calcCurrent(buffer0to32(readBuffer))*_calibration[channel][_I];
}


bool STPM::checkGain(uint8_t channel, uint8_t *buffer) {
  // Set current gain: 0x00 = 2, 0x01 = 4, 0x02 = 8, 0x03 = 16
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info:checkCurrentGain: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return false;
  }
  if (channel == 1) {
    if (DFE_CR101bits.LSB != buffer[2]) return false;
    if (DFE_CR101bits.MSB != buffer[3]) return false;
    if (DFE_CR101bits.GAIN1 != _gain1) return false;
  } else {
    if (DFE_CR201bits.LSB != buffer[2]) return false;
    if (DFE_CR201bits.MSB != buffer[3]) return false;
    if (DFE_CR201bits.GAIN1 != _gain2) return false;
  }
  return true;
}

void STPM::setCurrentGain(uint8_t channel, Gain gain) {
  // Set current gain: 0x00 = 2, 0x01 = 4, 0x02 = 8, 0x03 = 16
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info:setCurrentGain: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return;
  }
  uint8_t readAdd, writeAdd, dataLSB, dataMSB;


  if (channel == 1) {
    // Read Gain
    readFrame(0x18, readBuffer);
    //set GAIN1, in DFE_CR1 register
    readAdd = 0x00;
    writeAdd = 0x19;
    DFE_CR101bits.LSB = readBuffer[2];
    DFE_CR101bits.MSB = readBuffer[3];
    DFE_CR101bits.GAIN1 = gain;
    _gain1 = gain;
    dataLSB = DFE_CR101bits.LSB;
    dataMSB = DFE_CR101bits.MSB;    //set current gain for Chn1
  } else {
    // TODO: where is gain2? Wrong address here
    // Read Gain
    readFrame(0x1A, readBuffer);
    //set GAIN2 in DFE_CR2 register
    readAdd = 0x00;
    writeAdd = 0x1B;
    DFE_CR201bits.LSB = readBuffer[0];
    DFE_CR201bits.MSB = readBuffer[1];
    DFE_CR201bits.GAIN1 = gain;
    _gain2 = gain;
    dataLSB = DFE_CR201bits.LSB;
    dataMSB = DFE_CR201bits.MSB;
  }
  sendFrameCRC(readAdd, writeAdd, dataLSB, dataMSB);
}


void STPM::readPeriods(float* ch1, float* ch2) {
  if (!_autoLatch) latch();
  uint8_t address = Period_Address;
  readFrame(address, readBuffer);
  *ch1 = calcPeriod(buffer0to11(readBuffer));
  *ch2 = calcPeriod(buffer16to27(readBuffer));
}



void STPM::updateEnergy(uint8_t channel) {
  if (!_autoLatch) latch();
  if (channel > 2) return;
  int8_t address;
  Energy *energy = energies[channel];
  EnergyHelper *energyHelper = energiesHp[channel];
  if (channel == 0) {
    address = Tot_Active_Energy_Address;
  } else if (channel == 1) {
    address = PH1_Active_Energy_Address;
  } else if (channel == 2) {
    address = PH2_Active_Energy_Address;
  } 
  uint32_t myEnergies[4] = {0};
  SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);
  //delayMicroseconds(500);
  SPI.transfer(address);
  SPI.transfer(0xff);
  SPI.transfer(0xff);
  SPI.transfer(0xff);
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(4);
  digitalWrite(CS_PIN, LOW);
  for (int i = 0; i < 4; i++) {
    readBuffer[0] = SPI.transfer(0xff);
    readBuffer[1] = SPI.transfer(0xff);
    readBuffer[2] = SPI.transfer(0xff);
    readBuffer[3] = SPI.transfer(0xff);
    myEnergies[i] = (((readBuffer[3] << 24) | (readBuffer[2] << 16)) | (readBuffer[1] << 8)) | readBuffer[0];
  }
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  
  uint32_t now = millis();
  double * updateEnergies[4] = {&energy->active, &energy->fundamental, &energy->reactive, &energy->apparent};
  for (int i = 0; i < 4; i++) {
    // Calc delta and convert using LSB energy value
    uint32_t delta = myEnergies[i] - energyHelper->oldEnergy[i];
    double e = calcEnergy(delta);
    // calculate average power since last energy calc
    uint32_t deltaT = now-energy->validMillis;
    double p = e * 1000.0*3600.0/(double)deltaT;

    // If in positive direction
    if (delta < (uint32_t)2147483648) { // 2^31
      // If average power is below threshold treat as noise
      // If it is infeasible value, also do not use it
      // Otherwise increase energy
      if (p < NOISE_POWER || p > MAX_POWER) e = 0;
    } else {
      delta = energyHelper->oldEnergy[i] - myEnergies[i];
      e = -1.0*calcEnergy(delta);
      p = e * 1000.0*3600.0/(double)deltaT;
      if (p > -2*NOISE_POWER || p < -MAX_POWER) e = 0;
    }
    *updateEnergies[i] += e;
    energyHelper->oldEnergy[i] = myEnergies[i];

    #ifdef DEBUG_DEEP
    if (_logFunc) {
      _logFunc("%s Energy Calc: ", energyNames[i]);
      if (e < 0) _logFunc("negative energy");
      _logFunc("OldValue Value: %lu", energyHelper->oldEnergy[i]);
      _logFunc("Reg Value: %lu", myEnergies[i]);
      _logFunc("Delta Energy: %lu", delta);
      _logFunc("Delta Energy: %f", (float)e);
      _logFunc("Avg Power = %f", (float)p);
      _logFunc("Energy %.8f\n", (float)*updateEnergies[i]);
    }
    #endif
  }
  energy->validMillis = now;
}

double STPM::readTotalActiveEnergy() {
  return readActiveEnergy(0);
}

double STPM::readTotalFundamentalEnergy() {
  return readFundamentalEnergy(0);
}

double STPM::readTotalReactiveEnergy() {
  return readReactiveEnergy(0);
}

double STPM::readTotalApparentEnergy() {
  return readApparentEnergy(0);
}

double STPM::readActiveEnergy(uint8_t channel) {
  if (channel > 2) {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info:readActiveEnergy: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  // Check if it is time to update the energy first
  if (millis()-energies[channel]->validMillis > ENERGY_UPDATE_MS) updateEnergy(channel);
  return (energies[channel]->active);
}

double STPM::readFundamentalEnergy(uint8_t channel) {
  if (channel > 2) {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info:readFundamentalEnergy: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  // Check if it is time to update the energy first
  if (millis()-energies[channel]->validMillis > ENERGY_UPDATE_MS) updateEnergy(channel);
  return (energies[channel]->fundamental);
}

double STPM::readReactiveEnergy(uint8_t channel) {
  if (channel > 2) {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info:readReactiveEnergy: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  // Check if it is time to update the energy first
  if (millis()-energies[channel]->validMillis > ENERGY_UPDATE_MS) updateEnergy(channel);
  return (energies[channel]->reactive);
}

double STPM::readApparentEnergy(uint8_t channel) {
  if (channel > 2) {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info:readApparentEnergy: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  // Check if it is time to update the energy first
  if (millis()-energies[channel]->validMillis > ENERGY_UPDATE_MS) updateEnergy(channel);
  return (energies[channel]->apparent);
}

void STPM::resetEnergies() {
  uint32_t now = millis();
  totalEnergy = {0.0, 0.0, 0.0, 0.0, now};
  ph1Energy = {0.0, 0.0, 0.0, 0.0, now};
  ph2Energy = {0.0, 0.0, 0.0, 0.0, now};
}

bool STPM::checkEnergyOvf(uint8_t channel, char ** rawBuffer) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info:readApparentEnergy: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  uint8_t address = DSP_Status_1_Address;
  readFrame(address, readBuffer);

  if (rawBuffer) *rawBuffer = registerToStr(readBuffer);

  DSP_SR100bits.LSB = readBuffer[0];
  DSP_SR100bits.MSB = readBuffer[1];
  DSP_SR101bits.LSB = readBuffer[2];
  DSP_SR101bits.MSB = readBuffer[3];

  // Serial.println("Before Write:");
  // Serial.println(registerToStr(readBuffer));
  if (_logFunc) {
    // _logFunc("SR 1:");
    // _logFunc("\n%s", registerToStr(readBuffer));
    // address = DSP_Status_2_Address;
    // readFrame(address, readBuffer);
    // _logFunc("SR 2:");
    // _logFunc("\n%s", registerToStr(readBuffer));
    _logFunc("OVF Bits: All A: %i R: %i", DSP_SR100bits.TOTAL_ACTIVE_ENERGY_OVF, DSP_SR100bits.TOTAL_REACTIVE_ENERGY_OVF);
    _logFunc("OVF Bits: PH1 A: %i R: %i S: %i F: %i", DSP_SR101bits.PH1_ACTIVE_ENERGY_OVF, DSP_SR101bits.PH1_REACTIVE_ENERGY_OVF,
          DSP_SR101bits.PH1_FUNDAMENTAL_ENERGY_OVF, DSP_SR101bits.PH1_APPARENT_ENERGY_OVF);
    _logFunc("OVF Bits: PH2 A: %i R: %i S: %i F: %i", DSP_SR100bits.PH2_ACTIVE_ENERGY_OVF, DSP_SR100bits.PH2_REACTIVE_ENERGY_OVF,
          DSP_SR100bits.PH2_FUNDAMENTAL_ENERGY_OVF, DSP_SR100bits.PH2_APPARENT_ENERGY_OVF);
  }
  if (DSP_SR101bits.PH1_ACTIVE_ENERGY_OVF or DSP_SR100bits.PH2_ACTIVE_ENERGY_OVF or DSP_SR100bits.TOTAL_ACTIVE_ENERGY_OVF) {
    DSP_SR100bits.LSB = 0;
    DSP_SR100bits.MSB = 0;
    DSP_SR101bits.MSB = 0;
    DSP_SR101bits.LSB = 0;
    sendFrame(0xff, DSP_Status_1_Address, DSP_SR100bits.LSB, DSP_SR100bits.MSB);
    sendFrame(0xff, DSP_Status_1_Address+1,DSP_SR101bits.LSB, DSP_SR101bits.MSB);
    if (_logFunc) _logFunc("Ovf Bit(s) cleared");
  }
  // Serial.println("After Write:");
  // Serial.println(registerToStr(readBuffer));

  return true;
}

void STPM::readPower(uint8_t channel, float* active, float* fundamental, float* reactive, float* apparent) {
  if (channel == 1) {
    address = PH1_Active_Power_Address;
  } else if (channel == 2) {
    address = PH2_Active_Power_Address;
  } else {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return;
  }
  if (!_autoLatch) latch();

  sendFrame(address, 0xff, 0xff, 0xff);
  SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);
  //delayMicroseconds(1000);
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  *active = calcPower(buffer0to28(readBuffer))*_calibration[channel][_V]*_calibration[channel][_I];
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  *fundamental = calcPower(buffer0to28(readBuffer))*_calibration[channel][_V]*_calibration[channel][_I];
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  *reactive = calcPower(buffer0to28(readBuffer))*_calibration[channel][_V]*_calibration[channel][_I];
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  digitalWrite(CS_PIN, HIGH);
  *apparent = calcPower(buffer0to28(readBuffer))*_calibration[channel][_V]*_calibration[channel][_I];
  SPI.endTransaction();
}

float STPM::readActivePower(uint8_t channel) {
  if (channel == 1) { 
    address = PH1_Active_Power_Address;
  } else if (channel == 2) {
    address = PH2_Active_Power_Address;
  } else {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  if (!_autoLatch) latch();

  readFrame(address, readBuffer);
  return calcPower(buffer0to28(readBuffer))*_calibration[channel][_V]*_calibration[channel][_I];
}

float STPM::readFundamentalPower(uint8_t channel) {
  
  if (channel == 1) { 
    address = PH1_Fundamental_Power_Address;
  } else if (channel == 2) {
    address = PH2_Fundamental_Power_Address;
  } else {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  if (!_autoLatch) latch();

  readFrame(address, readBuffer);
  return (calcPower(buffer0to28(readBuffer)))*_calibration[channel][_V]*_calibration[channel][_I];
}

float STPM::readReactivePower(uint8_t channel) {
 
  if (channel == 1) { 
    address = PH1_Reactive_Power_Address;
  } else if (channel == 2) {
    address = PH2_Reactive_Power_Address;
  } else {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  if (!_autoLatch) latch();

  readFrame(address, readBuffer);
  return (calcPower(buffer0to28(readBuffer)))*_calibration[channel][_V]*_calibration[channel][_I];
}

float STPM::readApparentRMSPower(uint8_t channel) {
  
  if (channel == 1) { 
    address = PH1_Apparent_RMS_Power_Address;
  } else if (channel == 2) {
    address = PH2_Apparent_RMS_Power_Address;
  } else {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  if (!_autoLatch) latch();

  readFrame(address, readBuffer);
  return (calcPower(buffer0to28(readBuffer)))*_calibration[channel][_V]*_calibration[channel][_I];
}

float STPM::readApparentVectorialPower(uint8_t channel) {
  
  if (channel == 1) { 
    address = PH1_Apparent_Vectorial_Power_Address;
  } else if (channel == 2) {
    address = PH2_Apparent_Vectorial_Power_Address;
  } else {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  if (!_autoLatch) latch();

  readFrame(address, readBuffer);
  return (calcPower(buffer0to28(readBuffer)))*_calibration[channel][_V]*_calibration[channel][_I];
}

float STPM::readMomentaryActivePower(uint8_t channel) {

  if (channel == 1) { 
    address = PH1_Momentary_Active_Power_Address;
  } else if (channel == 2) {
    address = PH2_Momentary_Active_Power_Address;
  } else {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  if (!_autoLatch) latch();

  readFrame(address, readBuffer);
  return (calcPower(buffer0to28(readBuffer)))*_calibration[channel][_V]*_calibration[channel][_I];
}

float STPM::readMomentaryFundamentalPower(uint8_t channel) {
  
  if (channel == 1) { 
    address = PH1_Momentary_Fundamental_Power_Address;
  } else if (channel == 2) {
    address = PH2_Momentary_Fundamental_Power_Address;
  } else {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  if (!_autoLatch) latch();

  readFrame(address, readBuffer);
  return (calcPower(buffer0to28(readBuffer)))*_calibration[channel][_V]*_calibration[channel][_I];
}

// Overvoltage/overcurrent SWELL and undervoltage SAG
void STPM::readCurrentPhaseAndSwellTime(uint8_t channel, float* phase, float* swell) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info:readCurrentPhaseAndSwellTime: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    *phase = -1;
    *swell = -1;
    return;
  }
  uint8_t address = C1PHA_SWC1_TIME_Address;
  if (channel == 2) address = C2PHA_SWC2_TIME_Address;
  readFrame(address, readBuffer);
  *swell = (buffer0to14(readBuffer));
  *phase = (float)(buffer16to27(readBuffer))*netFreq*360.0/125000.0;
}

// Overvoltage/overcurrent SWELL and undervoltage SAG
void STPM::readVoltageSagAndSwellTime(uint8_t channel, float* sag, float* swell) {
  if (!_autoLatch) latch();
  if (channel != 1 && channel != 2) {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info:readSagAndSwellTime: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    *sag = -1;
    *swell = -1;
    return;
  }
  uint8_t address = SAG1_SWV1_TIME_Address;
  if (channel == 2) address = SAG2_SWV2_TIME_Address;
  readFrame(address, readBuffer);
  *swell = (buffer0to14(readBuffer));
  *sag = (buffer16to30(readBuffer));
}

int32_t value = 0;
void STPM::readVoltAndCurr(float *data) {
    if (!_autoLatch) {
      #ifdef SPI_LATCH
        DSP_CR301bits.SW_Latch1 = 1;      // Register latch
        DSP_CR301bits.SW_Latch2 = 1;      // Register latch
        SPI.beginTransaction(spiSettings);
        digitalWrite(CS_PIN, LOW);
        SPI.transfer(0x05);
        SPI.transfer(0x05);
        SPI.transfer(DSP_CR301bits.LSB);
        SPI.transfer(DSP_CR301bits.MSB);
        //delayMicroseconds(5);
        digitalWrite(CS_PIN, HIGH);
        SPI.endTransaction();
        // TODO: Required, see datasheet?
        delayMicroseconds(4);
      #else
        digitalWrite(SYN_PIN, LOW);
        delayMicroseconds(4);
        digitalWrite(SYN_PIN, HIGH);
        delayMicroseconds(4);
      #endif
    }
    address = V1_Data_Address;
    uint8_t channel = 1;
    SPI.beginTransaction(spiSettings);
    digitalWrite(CS_PIN, LOW);
    //delayMicroseconds(500);
    SPI.transfer(address);
    SPI.transfer(0xff);
    SPI.transfer(0xff);
    SPI.transfer(0xff);
    digitalWrite(CS_PIN, HIGH);
    delayMicroseconds(4);
    digitalWrite(CS_PIN, LOW);
    readBuffer[0] = SPI.transfer(0xff);
    readBuffer[1] = SPI.transfer(0xff);
    readBuffer[2] = SPI.transfer(0xff);
    readBuffer[3] = SPI.transfer(0xff);
    value = (((readBuffer[3] << 24) | (readBuffer[2] << 16)) | (readBuffer[1] << 8)) | readBuffer[0];
    data[0] = ((float)value/*-14837*/)*0.000138681*_calibration[channel][_V];
    readBuffer[0] = SPI.transfer(0xff);
    readBuffer[1] = SPI.transfer(0xff);
    readBuffer[2] = SPI.transfer(0xff);
    readBuffer[3] = SPI.transfer(0xff);
    digitalWrite(CS_PIN, HIGH);
    value = (((readBuffer[3] << 24) | (readBuffer[2] << 16)) | (readBuffer[1] << 8)) | readBuffer[0];
    data[1] = ((float)value/*-14837*/)* 0.003349213*_calibration[channel][_I];

    SPI.endTransaction();
}

void STPM::readVoltageAndCurrent(uint8_t channel, float *voltage, float *current) {
  
  if (channel == 1) { 
    address = V1_Data_Address;
  } else if (channel == 2) {
    address = V2_Data_Address;
  } else {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return;
  }
  if (!_autoLatch) latch();
  
  sendFrame(address, 0xff, 0xff, 0xff);
  SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);
  //delayMicroseconds(1000);
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  *voltage = calcVolt(buffer0to32(readBuffer))*_calibration[channel][_V];
  readBuffer[0] = SPI.transfer(0xff);
  readBuffer[1] = SPI.transfer(0xff);
  readBuffer[2] = SPI.transfer(0xff);
  readBuffer[3] = SPI.transfer(0xff);
  digitalWrite(CS_PIN, HIGH);
  *current = calcCurrent(buffer0to32(readBuffer))*_calibration[channel][_I];
  SPI.endTransaction();
}

float STPM::readVoltage(uint8_t channel) {
  
  if (channel == 1) { 
    address = V1_Data_Address;
  } else if (channel == 2) {
    address = V2_Data_Address;
  } else {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  if (!_autoLatch) latch();
  
  readFrame(address, readBuffer);
  return (calcVolt((int32_t)buffer0to32(readBuffer)))*_calibration[channel][_V];
}


float STPM::readCurrent(uint8_t channel) {
  
  if (channel == 1) { 
    address = V1_Data_Address;
  } else if (channel == 2) {
    address = V2_Data_Address;
  } else {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  if (!_autoLatch) latch();
  

  readFrame(address, readBuffer);
  return (calcCurrent((int32_t)buffer0to32(readBuffer)))*_calibration[channel][_I];
}


float STPM::readFundamentalVoltage(uint8_t channel) {
  
  if (channel == 1) { 
    address = V1_Fund_Address;
  } else if (channel == 2) {
    address = V2_Fund_Address;
  } else {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  if (!_autoLatch) latch();

  readFrame(address, readBuffer);
  return (calcVolt(buffer0to32(readBuffer)))*_calibration[channel][_V];
}

void STPM::readRMSVoltageAndCurrent(uint8_t channel, float* voltage, float* current) {
  
  if (channel == 1) { 
    address = C1_RMS_Data_Address;
  } else if (channel == 2) {
    address = C2_RMS_Data_Address;
  } else {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return;
  }
  if (!_autoLatch) latch();

  readFrame(address, readBuffer);
  *voltage = calcVolt((uint16_t)buffer0to14(readBuffer))*_calibration[channel][_V];
  *current = calcCurrent((uint32_t)buffer15to32(readBuffer))*_calibration[channel][_I];
}

float STPM::readRMSVoltage(uint8_t channel) {
  
  if (channel == 1) { 
    address = C1_RMS_Data_Address;
  } else if (channel == 2) {
    address = C2_RMS_Data_Address;
  } else {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  if (!_autoLatch) latch();

  readFrame(address, readBuffer);
  return calcVolt((uint16_t)buffer0to14(readBuffer))*_calibration[channel][_V];
}

float STPM::readRMSCurrent(uint8_t channel) {
  
  if (channel == 1) { 
    address = C1_RMS_Data_Address;
  } else if (channel == 2) {
    address = C2_RMS_Data_Address;
  } else {
    #ifdef DEBUG_DEEP
    Serial.print(F("Info: Channel "));
    Serial.print(channel);
    Serial.println(F(" out of range"));
    #endif
    return -1;
  }
  if (!_autoLatch) latch();

  readFrame(address, readBuffer);
  return calcCurrent((uint32_t)buffer15to32(readBuffer))*_calibration[channel][_I];
}

/*
* Latch a new measurement in the registers. CS_PIN should be high and automatic
* measurement off.
*/
void STPM::latchReg() {
	latch();
}

/*
* Latch a new measurement in the registers. CS_PIN should be high and automatic
* measurement off. As inline function.
*/
inline void STPM::latch() {
#ifdef SPI_LATCH
  DSP_CR301bits.SW_Latch1 = 1;      // Register latch
  DSP_CR301bits.SW_Latch2 = 1;      // Register latch
  sendFrame(0x05, 0x05, DSP_CR301bits.LSB, DSP_CR301bits.MSB);
#else
  digitalWrite(SYN_PIN, LOW);
  delayMicroseconds(4);
  digitalWrite(SYN_PIN, HIGH);
  delayMicroseconds(4);
#endif
}

/* Conversion of the register values into voltage, current, power, etc
*  see: STPM3x design and calibration guideline for customers v3.1.xlsm
*/

// In Hz
inline float STPM::calcPeriod (uint16_t value) {
  return 1.0/((float)value * 8.0/1000000.0);
}
/* Returns power in W (Power register LSB)
*  LSBP = (1+R1/R2) * Vref^2 / (Ks * kint * Av * Ai * calV  * calI * 2^28)
*  R1 810 KOhm, R2 470 Ohm, Vref = 1180 mV, Ks = 3 mOhm, Kint = 1,
*  Av = 2, Ai = 16, calV = 0.875, calI = 0.875
*/
inline float STPM::calcPower (int32_t value) {
  return value * 0.0001217; //0.000152; old value from Benny.
}
/* Returns Energy in Ws (Energy register LSB)
*  LSBE = (1+R1/R2) * Vref^2 / (Ks * kint * Av * Ai * calV  * calI * 2^17 * Fs)
*  R1 810KOhm, R2 470 Ohm, Vref = 1180 mV, Ks = 3 mOhm, Kint = 1,
*  Av = 2, Ai = 16, calV = 0.875, calI = 0.875, Fs = 7812,5Hz
*/
inline double STPM::calcEnergy (uint32_t value) {
  // The heck is this value, with the formula i calculated sth else
  // return (float)value * 1./3600; // maybe this value is 10x to small? test it. // This is watt seconds, we want watt hours
  //0.00040; old value from Benny
  return (double)value * ENERGY_LSB;
}

/* Retruns current in mA (LSB current)
*  LSBC = Vref   /  (calI* Ai * 2^23 * ks * kint)
*  Vref = 1180 mV, Ks = 3 mOhm, calI = 0.875, Kint = 1, Ai = 16
*/
inline float STPM::calcCurrent (int32_t value) {
  return (float)value * 0.003349213; //1000; // 0.0041875 old value from Benny
}

/* Retruns current in mA (LSB current)
*  LSBC = Vref   /  (calI* Ai * 2^17 * ks * kint)
*  Vref = 1180 mV, Ks = 3 mOhm, calI = 0.875, Kint = 1, Ai = 16
*/
inline float STPM::calcCurrent (uint32_t value) {
  return (float)value * 0.2143; //1000; // 0.26794 old value from Benny
}

/* This value is for R1 810 KOhm R2 470 Ohm
*  Instantaneous LSB voltage  = Vref*(1+R1/R2) / (calV * Av * 2^23)
*  with calV = 0.875 and Av = 2
*  retuns Voltage in V
*/
inline float STPM::calcVolt (int32_t value) {
  return ((float)value/*-14837*/)*0.000138681; ///7186.4; (from Benny function unknown)
}

/* This value is for R1 810 KOhm R2 470 Ohm
*  LSB voltage= Vref*(1+R1/R2) / (calV * Av * 2^15)
*  with Vref = 1180 mV, calV = 0.875 and Av = 2
*  retuns Voltage in V
*/
inline float STPM::calcVolt (uint16_t value) {
  return ((float)value/*-56*/)* 0.0354840440; // old value from Benny  0.03550231588
}

inline int32_t STPM::buffer0to32(uint8_t *buffer) {
  return (((buffer[3] << 24) | (buffer[2] << 16)) | (buffer[1] << 8)) | buffer[0];
}
inline int32_t STPM::buffer0to28(uint8_t *buffer) {
  return (((buffer[3] << 24) | (buffer[2] << 16)) | (buffer[1] << 8)) | buffer[0];
}
inline uint32_t STPM::buffer15to32(uint8_t *buffer) {
  return (((buffer[3] << 16) | (buffer[2] << 8)) | buffer[1]) >> 7;
}
inline uint16_t STPM::buffer16to30(uint8_t *buffer) {
  return ((buffer[3]&0x7f) << 8) | buffer[2];
}
inline uint16_t STPM::buffer0to14(uint8_t *buffer) {
  return ((buffer[1]&0x7f) << 8) | buffer[0];
}
inline uint16_t STPM::buffer0to11(uint8_t *buffer) {
  return ((buffer[1]&0x0f) << 8) | buffer[0];
}
inline uint16_t STPM::buffer16to27(uint8_t *buffer) {
  return ((buffer[3]&0x0f) << 8) | buffer[2];
}

void STPM::readFrame(uint8_t address, uint8_t *buffer) {
  sendFrame(address, 0xff, 0xff, 0xff);
  SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);
  //delayMicroseconds(5);
  for (uint8_t i = 0; i < 4; i++) {
    buffer[i] = SPI.transfer(0xff);
  }
  //delayMicroseconds(5);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}


void STPM::sendFrame(uint8_t readAdd, uint8_t writeAdd, uint8_t dataLSB, uint8_t dataMSB) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);
  //delayMicroseconds(5);
  SPI.transfer(readAdd);
  SPI.transfer(writeAdd);
  SPI.transfer(dataLSB);
  SPI.transfer(dataMSB);
  //delayMicroseconds(5);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}

void STPM::sendFrameCRC(uint8_t readAdd, uint8_t writeAdd, uint8_t dataLSB, uint8_t dataMSB) {
  uint8_t frame[STPM3x_FRAME_LEN];
  frame[0] = readAdd;
  frame[1] = writeAdd;
  frame[2] = dataLSB;
  frame[3] = dataMSB;
  frame[4] = CalcCRC8(frame);
#ifdef DEBUG_DEEP
  printRegister(frame, "Sending:");
#endif
  SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);
  //delayMicroseconds(4);
  SPI.transfer(frame[0]);
  SPI.transfer(frame[1]);
  SPI.transfer(frame[2]);
  SPI.transfer(frame[3]);
  SPI.transfer(frame[4]);
  //delayMicroseconds(4);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}

void STPM::printFrame(uint8_t *frame, uint8_t length) {
  char buffer[4];
  Serial.print(F("Info:"));
  for (uint8_t i = 0; i < length; i++) {
    snprintf(buffer, 4, "%02x", frame[i]);
    Serial.print(F("|"));
    Serial.print(buffer);
  }
  Serial.println(F("|"));
}

char * STPM::registerToStr(uint8_t *frame) {
  strncpy(testStr, "", sizeof(testStr));
  char * str = testStr;
  str += snprintf(str, TEST_STR_SIZE, "|"); 
  for (int8_t i = 31; i >= 0; i--) {
    str += snprintf(str, TEST_STR_SIZE-strlen(testStr), "%02i|", i);
  }
  str += snprintf(str, TEST_STR_SIZE-strlen(testStr), "\n| ");
  for (int8_t i = 3; i >= 0; i--) {
    str += snprintf(str, TEST_STR_SIZE-strlen(testStr), "%s| ", (frame[i] & 0x80) > 0 ? "1" : "0");
    str += snprintf(str, TEST_STR_SIZE-strlen(testStr), "%s| ", (frame[i] & 0x40) > 0 ? "1" : "0");
    str += snprintf(str, TEST_STR_SIZE-strlen(testStr), "%s| ", (frame[i] & 0x20) > 0 ? "1" : "0");
    str += snprintf(str, TEST_STR_SIZE-strlen(testStr), "%s| ", (frame[i] & 0x10) > 0 ? "1" : "0");
    str += snprintf(str, TEST_STR_SIZE-strlen(testStr), "%s| ", (frame[i] & 0x08) > 0 ? "1" : "0");
    str += snprintf(str, TEST_STR_SIZE-strlen(testStr), "%s| ", (frame[i] & 0x04) > 0 ? "1" : "0");
    str += snprintf(str, TEST_STR_SIZE-strlen(testStr), "%s| ", (frame[i] & 0x02) > 0 ? "1" : "0");
    str += snprintf(str, TEST_STR_SIZE-strlen(testStr), "%s| ", (frame[i] & 0x01) > 0 ? "1" : "0");
  }
  return testStr;
}

void STPM::printRegister(uint8_t *frame, const char* regName) {
  Serial.print(F("Info:"));
  Serial.println(regName);
  Serial.print(F("Info:"));
  Serial.print(F("|"));
  for (int8_t i = 31; i >= 0; i--) {
    Serial.print(i);
    Serial.print(F("|"));
    if (i <= 10) Serial.print(" ");
  }
  Serial.println("");
  Serial.print(F("Info:"));
  Serial.print(F("| "));
  for (int8_t i = 3; i >= 0; i--) {
    Serial.print((frame[i] & 0x80) > 0);Serial.print(F("| "));
    Serial.print((frame[i] & 0x40) > 0);Serial.print(F("| "));
    Serial.print((frame[i] & 0x20) > 0);Serial.print(F("| "));
    Serial.print((frame[i] & 0x10) > 0);Serial.print(F("| "));
    Serial.print((frame[i] & 0x08) > 0);Serial.print(F("| "));
    Serial.print((frame[i] & 0x04) > 0);Serial.print(F("| "));
    Serial.print((frame[i] & 0x02) > 0);Serial.print(F("| "));
    Serial.print((frame[i] & 0x01) > 0);Serial.print(F("| "));
  }
  Serial.println("");
}

uint8_t STPM::CalcCRC8(uint8_t *pBuf) {
  uint8_t i;
  CRC_u8Checksum = 0x00;
  for (i = 0; i < STPM3x_FRAME_LEN - 1; i++) {
    Crc8Calc(pBuf[i]);
  }
  return CRC_u8Checksum;
}

void STPM::Crc8Calc(uint8_t u8Data) {
  uint8_t loc_u8Idx;
  uint8_t loc_u8Temp;
  loc_u8Idx = 0;
  while (loc_u8Idx < 8) {
    loc_u8Temp = u8Data ^ CRC_u8Checksum;
    CRC_u8Checksum <<= 1;
    if (loc_u8Temp & 0x80) {
      CRC_u8Checksum ^= CRC_8;
    }
    u8Data <<= 1;
    loc_u8Idx++;
  }
}
