/***************************************************
 Library for getting energy-data out of a STPM3X.
 See example file to get an idea of how to use this
 class.

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#ifndef STPM_h
#define STPM_h


#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <SPI.h>
#include "STPM3X_DEFINE.h"
// #define DEBUG_DEEP


#define STPM3x_FRAME_LEN 5
#define CRC_8 (0x07)
#define SPI_LATCH // Latching with SPI
#define _V 0
#define _I 1

enum Gain { twoX=0x00, fourX=0x01, eightX=0x02, sixteenX=0x03};

class STPM {
  public:
    STPM(int resetPin, int csPin, int synPin);
    STPM(int resetPin, int csPin);
    bool init();
    void setCalibration(float calV, float calI);
    void setCalibration(float * calibration);
    void setCurrentGain(uint8_t channel, Gain gain);
    bool checkGain(uint8_t channel, uint8_t *buffer);
    float readTotalActiveEnergy();
    float readTotalFundamentalEnergy();
    float readTotalReactiveEnergy();
    float readTotalApparentEnergy();
    float readActiveEnergy(uint8_t channel);
    float readFundamentalEnergy(uint8_t channel);
    float readReactiveEnergy(uint8_t channel);
    float readApparentEnergy(uint8_t channel);
    void readPower(uint8_t channel, float* active, float* fundamental, float* reactive, float* apparent);
    float readActivePower(uint8_t channel);
    float readFundamentalPower(uint8_t channel);
    float readReactivePower(uint8_t channel);
    float readApparentRMSPower(uint8_t channel);
    float readApparentVectorialPower(uint8_t channel);
    float readMomentaryActivePower(uint8_t channel);
    float readMomentaryFundamentalPower(uint8_t channel);
#if defined(ESP32) or defined(ESP8266) 
    void IRAM_ATTR readAll(uint8_t channel, float *voltage, float *current, float* active, float* reactive);
    void IRAM_ATTR readVoltageAndCurrent(uint8_t channel, float* voltage, float* current);
    void IRAM_ATTR readVoltAndCurr(float* data);
    void IRAM_ATTR latchReg();
#else
    void readAll(uint8_t channel, float *voltage, float *current, float* active, float* reactive);
    void readVoltageAndCurrent(uint8_t channel, float* voltage, float* current);
    void readVoltAndCurr(float* data);
    void latchReg();
#endif
    float readVoltage(uint8_t channel);
    float readCurrent(uint8_t channel);
    float readFundamentalVoltage(uint8_t channel);
    void readRMSVoltageAndCurrent(uint8_t channel, float* voltage, float* current);
    float readRMSVoltage(uint8_t channel);
    float readRMSCurrent(uint8_t channel);
    void readVoltageSagAndSwellTime(uint8_t channel, float* sag, float* swell);
    void readCurrentPhaseAndSwellTime(uint8_t channel, float* phase, float* swell);
    void readPeriods(float* ch1, float* ch2);
    void autoLatch(bool enabled);
    void CRC(bool enabled);

  private:
    bool Init_STPM34();
#if defined(ESP32) or defined(ESP8266) 
    void IRAM_ATTR readFrame(uint8_t address, uint8_t *buffer);
    void IRAM_ATTR sendFrame(uint8_t readAdd, uint8_t writeAdd, uint8_t dataLSB, uint8_t dataMSB);
#else
    void readFrame(uint8_t address, uint8_t *buffer);
    void sendFrame(uint8_t readAdd, uint8_t writeAdd, uint8_t dataLSB, uint8_t dataMSB);
#endif
    void sendFrameCRC(uint8_t readAdd, uint8_t writeAdd, uint8_t dataLSB, uint8_t dataMSB);
    void printFrame(uint8_t *frame, uint8_t length);
    void printRegister(uint8_t *frame, const char* regName);
    uint8_t CalcCRC8(uint8_t *pBuf);
    void Crc8Calc (uint8_t u8Data);
    inline float calcPeriod (int16_t value);
    inline float calcVolt (int16_t value);
    inline float calcVolt (int32_t value);
    inline float calcCurrent (int16_t value);
    inline float calcCurrent (int32_t value);
    inline float calcEnergy (int32_t value);
    inline float calcPower (int32_t value);

    inline int32_t buffer0to32(uint8_t *buffer);
    inline int32_t buffer0to28(uint8_t *buffer);
    inline int16_t buffer0to14(uint8_t *buffer);
    inline int16_t buffer0to11(uint8_t *buffer);
    inline int32_t buffer15to32(uint8_t *buffer);
    inline int16_t buffer16to30(uint8_t *buffer);
    inline int16_t buffer16to27(uint8_t *buffer);

    inline void latch();

    float _calibration[3][2];
    int RESET_PIN;
    int CS_PIN;
    int SYN_PIN;
    uint8_t CRC_u8Checksum;
    uint8_t address;
    bool _autoLatch;
    bool _crcEnabled;
    Gain _gain1;
    Gain _gain2;
    uint8_t readBuffer[10];

};

#endif
