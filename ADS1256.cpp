/*
  ADS1256.h - Arduino Library for communication with Texas Instrument ADS1256 ADC
  Written by Adien Akhmad, August 2015
  Modfified  Jan 2019 by Axel Sepulveda for ATMEGA328
  Modified July 2025 by John Freudenthal to work with alternative SPI buses and connection settings.
*/

#include "ADS1256.h"
#include "Arduino.h"
#include "SPI.h"
#include <cmath>

static const uint16_t calibrationTimeOut = 1'500'000; // 1.5 seconds in microseconds

// static class constants

const uint8_t ADS1256::DATA_RATE_CODES[ADS1256::NUM_DATA_RATES] = {
  ADS1256_DRATE_2_5SPS,
  ADS1256_DRATE_5SPS,
  ADS1256_DRATE_10SPS,
  ADS1256_DRATE_15SPS,
  ADS1256_DRATE_25SPS,
  ADS1256_DRATE_30SPS,
  ADS1256_DRATE_50SPS,
  ADS1256_DRATE_60SPS,
  ADS1256_DRATE_100SPS,
  ADS1256_DRATE_500SPS,
  ADS1256_DRATE_1000SPS,
  ADS1256_DRATE_2000SPS,
  ADS1256_DRATE_3750SPS,
  ADS1256_DRATE_7500SPS,
  ADS1256_DRATE_15000SPS,
  ADS1256_DRATE_30000SPS,
};
const float ADS1256::DATA_RATES[ADS1256::NUM_DATA_RATES] = {
	2.5,
	5,
	10,
	15,
	25,
	30,
	50,
	60,
	100,
	500,
	1'000,
	2'000,
	3'750,
	7'500,
	15'000,
	30'000,
};

// static class methods

uint8_t ADS1256::dataRateCodeFromValue(float dataRate)
{
  for (int i = 0; i < ADS1256::NUM_DATA_RATES; i++) {
    if (DATA_RATES[i] == dataRate)
    {
      return DATA_RATE_CODES[i];
    }
    else if (DATA_RATES[i] > dataRate)
    {
      break;
    }
  }
  return ADS1256_DRATE_NIL;
}

float ADS1256::dataRateValueFromCode(uint8_t dataRateCode)
{
  for (int i = 0; i < ADS1256::NUM_DATA_RATES; i++) {
    if (DATA_RATE_CODES[i] == dataRateCode)
    {
      return DATA_RATES[i];
    }
  }
  return 0;
}

// constructor

ADS1256::ADS1256(SPIClass* spi, float clockSpdMhz, float vref, uint8_t cspin, uint8_t readypin, uint8_t resetpin, uint8_t powerdownpin)
{
  SPIBus = spi;
  // according to ADS datasheet, SPI CLK Tperiod must be within 4 Tclk to 10 Tdata,
  // which means max SPI CLK freq is 1/4th clockSpeed and min SPI CLK freq is 1/10 datarate
  uint32_t SPISpeed = clockSpdMhz * 1e6 / 4;
  ConnectionSettings = SPISettings(SPISpeed, MSBFIRST, SPI_MODE1);
  ReadyPin = readypin;
  ResetPin = resetpin;
  PowerDownPin = powerdownpin;
  CSPin = cspin;
  // Voltage Reference
  _VREF = vref;
  DelayT6 = (uint16_t)std::ceil(50.0 / clockSpdMhz);
  DelayT11_4 = (uint16_t)std::ceil(4.0 / clockSpdMhz);
  DelayT11_24 = (uint16_t)std::ceil(24.0 / clockSpdMhz);
  CSEnabled = false;
  
  // ready and chip select pins are not optional
  pinMode(ReadyPin, INPUT);
  pinMode(CSPin, OUTPUT);
  digitalWrite(CSPin, HIGH);
  // reset and power down pins are optional
  if (ResetPin != ADS1256_NOPIN)
  {
    pinMode(ResetPin, OUTPUT);
    digitalWrite(ResetPin, HIGH);
  }
  if (PowerDownPin != ADS1256_NOPIN)
  {
    pinMode(PowerDownPin, OUTPUT);
    digitalWrite(PowerDownPin, HIGH);
  }
}

/*
Init chip with set datarate and gain and perform self calibration
*/
bool ADS1256::begin(unsigned char drate, unsigned char gain, bool buffenable)
{
  _pga = 1 << gain;
  writeRegister(ADS1256_RADD_DRATE, drate); // write data rate register
  uint8_t gainBytemask = B00000111;
  uint8_t adcon = readRegister(ADS1256_RADD_ADCON);
  uint8_t byte2send = (adcon & ~gainBytemask) | gain;
  bitWrite(byte2send, 6, 0);
  bitWrite(byte2send, 5, 0);
  writeRegister(ADS1256_RADD_ADCON, byte2send);
  if (buffenable)
  {
    uint8_t status = readRegister(ADS1256_RADD_STATUS);
    bitSet(status, 1);
    writeRegister(ADS1256_RADD_STATUS, status);
  }
  return selfCalibrateAll(); // return if calibration is successful.
  // otherwise, waiting timed out, which would be odd.
}

// specific use command sequences

float ADS1256::ReadAndSwitchChannels(byte channel)
{
  float Value;
  bool wasOn = CSON();
  setChannel(channel);
  sync();
  Value = readDataFloat();
  if (!wasOn) CSOFF();
  return Value;
}

// Channel Switching for differential mode.
// Negative input channel are automatically set to AINCOM
void ADS1256::setChannel(byte AIN_P, byte AIN_N)
{
  unsigned char MUX_CHANNEL;
  unsigned char MUXP;
  unsigned char MUXN;
  switch (AIN_P)
  {
    case 0:
      MUXP = ADS1256_MUXP_AIN0;
      break;
    case 1:
      MUXP = ADS1256_MUXP_AIN1;
      break;
    case 2:
      MUXP = ADS1256_MUXP_AIN2;
      break;
    case 3:
      MUXP = ADS1256_MUXP_AIN3;
      break;
    case 4:
      MUXP = ADS1256_MUXP_AIN4;
      break;
    case 5:
      MUXP = ADS1256_MUXP_AIN5;
      break;
    case 6:
      MUXP = ADS1256_MUXP_AIN6;
      break;
    case 7:
      MUXP = ADS1256_MUXP_AIN7;
      break;
    default:
      MUXP = ADS1256_MUXP_AINCOM;
  }
  switch (AIN_N)
  {
    case 0:
      MUXN = ADS1256_MUXN_AIN0;
      break;
    case 1:
      MUXN = ADS1256_MUXN_AIN1;
      break;
    case 2:
      MUXN = ADS1256_MUXN_AIN2;
      break;
    case 3:
      MUXN = ADS1256_MUXN_AIN3;
      break;
    case 4:
      MUXN = ADS1256_MUXN_AIN4;
      break;
    case 5:
      MUXN = ADS1256_MUXN_AIN5;
      break;
    case 6:
      MUXN = ADS1256_MUXN_AIN6;
      break;
    case 7:
      MUXN = ADS1256_MUXN_AIN7;
      break;
    default:
      MUXN = ADS1256_MUXN_AINCOM;
  }
  MUX_CHANNEL = MUXP | MUXN;
  bool wasOn = CSON();
  writeRegister(ADS1256_RADD_MUX, MUX_CHANNEL);
  if (!wasOn) CSOFF();
}

// specific sub commands

/*
Reads and returns STATUS register
*/
uint8_t ADS1256::getStatus()
{
  return readRegister(ADS1256_RADD_STATUS);
}

void ADS1256::setDataRateCode(uint8_t dataRateCode)
{
  writeRegister(ADS1256_RADD_DRATE, dataRateCode);
}

uint8_t ADS1256::getDataRateCode()
{
  return readRegister(ADS1256_RADD_DRATE);
}

float ADS1256::getDataRate()
{
  return ADS1256::dataRateValueFromCode(getDataRateCode());
}

void ADS1256::GPIOPinMode(uint8_t Pin, uint8_t InOut)
{
  if (Pin > 3)
  {
    Pin = 3;
  }
  if (InOut > 1)
  {
    InOut = 0;
  }
  uint8_t DigitalIO = readRegister(ADS1256_RADD_IO);
  uint8_t BitToWrite = Pin + 4;
  bitWrite(DigitalIO, BitToWrite, InOut);
  writeRegister(ADS1256_RADD_IO, DigitalIO);
}

bool ADS1256::GPIOPinRead(uint8_t Pin)
{
  if (Pin > 3)
  {
    Pin = 3;
  }
  uint8_t DigitalIO = readRegister(ADS1256_RADD_IO);
  DigitalIO = readRegister(ADS1256_RADD_IO);
  return bitRead(DigitalIO,Pin);
}

void ADS1256::GPIOPinWrite(uint8_t Pin, bool OffOn)
{
  if (Pin > 3)
  {
    Pin = 3;
  }
  uint8_t DigitalIO = readRegister(ADS1256_RADD_IO);
  bitWrite(DigitalIO, Pin, OffOn);
  writeRegister(ADS1256_RADD_IO, DigitalIO);
}

// commands

void ADS1256::writeRegister(unsigned char reg, unsigned char wdata)
{
  bool wasOn = CSON();
  SPIBus->transfer(ADS1256_CMD_WREG | reg); // opcode1 Write registers starting from reg
  SPIBus->transfer(0); // opcode2 Write 1+0 registers
  SPIBus->transfer(wdata); // write wdata
  delayMicroseconds(DelayT11_4);
  if (!wasOn) CSOFF();
}

unsigned char ADS1256::readRegister(unsigned char reg)
{
  unsigned char readValue;
  bool wasOn = CSON();
  SPIBus->transfer(ADS1256_CMD_RREG | reg); // opcode1 read registers starting from reg
  SPIBus->transfer(0); // opcode2 read 1+0 registers
  delayMicroseconds(DelayT6); // t6 delay (4*tCLKIN 50*0.13 = 6.5 us)
  readValue = SPIBus->transfer(0); // read registers
  delayMicroseconds(DelayT11_4); // t11 delay (4*tCLKIN 4*0.13 = 0.52 us)
  if (!wasOn) CSOFF();
  return readValue;
}

float ADS1256::readDataFloat()
{
  double adsCode = readDataInt32();
  return ((adsCode * 2.0 * _VREF) / (0x7FFFFF * _pga));
}

// Reads raw ADC data, as 32bit int
long ADS1256::readDataInt32()
{
  bool wasOn = CSON();
  SPIBus->transfer(ADS1256_CMD_RDATA);
  delayMicroseconds(DelayT6); // t6 delay (4*tCLKIN 50*0.13 = 6.5 us)
  long adsCode = read_int32();
  delayMicroseconds(DelayT11_4); // t11 delay (4*tCLKIN 4*0.13 = 0.52 us)
  if (!wasOn) CSOFF();
  return adsCode;
}

void ADS1256::sync()
{
  bool wasOn = CSON();
  SPIBus->transfer(ADS1256_CMD_SYNC);
  delayMicroseconds(DelayT11_24);
  SPIBus->transfer(ADS1256_CMD_WAKEUP);
  if (!wasOn) CSOFF();
}

bool ADS1256::standby()
{
  bool wasOn = CSON();
  SPIBus->transfer(ADS1256_CMD_STANDBY);
  bool didntTimeOut = waitDRDY();
  if (!wasOn) CSOFF();
  return didntTimeOut;
}

void ADS1256::wakeup()
{
  bool wasOn = CSON();
  SPIBus->transfer(ADS1256_CMD_WAKEUP);
  if (!wasOn) CSOFF();
}

bool ADS1256::selfCalibrateAll()
{
  bool wasOn = CSON();
  SPIBus->transfer(ADS1256_CMD_SELFCAL);
  bool didntTimeOut = waitDRDY(calibrationTimeOut);
  if (!wasOn) CSOFF();
  return didntTimeOut;
}

bool ADS1256::selfCalibrateGain()
{
  bool wasOn = CSON();
  SPIBus->transfer(ADS1256_CMD_SELFGCAL);
  bool didntTimeOut = waitDRDY(calibrationTimeOut);
  if (!wasOn) CSOFF();
  return didntTimeOut;
}

bool ADS1256::selfCalibrateOffset()
{
  bool wasOn = CSON();
  SPIBus->transfer(ADS1256_CMD_SELFOCAL);
  bool didntTimeOut = waitDRDY(calibrationTimeOut);
  if (!wasOn) CSOFF();
  return didntTimeOut;
}

bool ADS1256::systemCalibrateGain()
{
  // system calibrations require relevant voltages to be supplied to the correct pins on the chip
  bool wasOn = CSON();
  SPIBus->transfer(ADS1256_CMD_SYSGCAL);
  bool didntTimeOut = waitDRDY(calibrationTimeOut);
  if (!wasOn) CSOFF();
  return didntTimeOut;
}

bool ADS1256::systemCalibrateOffset()
{
  // system calibrations require relevant voltages to be supplied to the correct pins on the chip
  bool wasOn = CSON();
  SPIBus->transfer(ADS1256_CMD_SYSOCAL);
  bool didntTimeOut = waitDRDY(calibrationTimeOut);
  if (!wasOn) CSOFF();
  return didntTimeOut;
}

// command helpers

// Call this ONLY after ADS1256_CMD_RDATA command
unsigned long ADS1256::read_uint24()
{
  unsigned char _highByte, _midByte, _lowByte;
  unsigned long value;
  _highByte = SPIBus->transfer(0);
  _midByte  = SPIBus->transfer(0);
  _lowByte  = SPIBus->transfer(0);
  // Combine all 3-bytes to 24-bit data using byte shifting.
  value = ((long)_highByte << 16) + ((long)_midByte << 8) + ((long)_lowByte);
  return value;
}

// Call this ONLY after ADS1256_CMD_RDATA command
// Convert the signed 24bit stored in an unsigned 32bit to a signed 32bit
long ADS1256::read_int32()
{
  long value = read_uint24();
  if (value & 0x00800000) // if the 24 bit value is negative reflect it to 32bit
  {
    value |= 0xff000000;
  }
  return value;
}

// spi settings

SPISettings* ADS1256::GetSPISettings()
{
  return &ConnectionSettings;
}

// pin sequence operations

bool ADS1256::CSON()
{
  bool wasOn = CSEnabled;
  if (!wasOn)
  {
    SPIBus->beginTransaction(ConnectionSettings);
    digitalWrite(CSPin, LOW);
    CSEnabled = true;
  }
  return wasOn;
}

void ADS1256::CSOFF()
{
  if (CSEnabled)
  {
    digitalWrite(CSPin, HIGH);
    SPIBus->endTransaction();
    CSEnabled = false;
  }
}

// default timeout of 0.5 seconds
bool ADS1256::waitDRDY(uint32_t timeoutMicros)
{
  bool KeepWaiting = true;
  uint32_t WaitStart = micros();
  while (KeepWaiting)
  {
    if (isDRDY())
    {
      KeepWaiting = false;
    }
    else
    {
      if ( (micros() - WaitStart) > timeoutMicros)
      {
        Serial.println("ADS1256 error on waitDRDY.");
        KeepWaiting = false;
        // rdy never went true, failed to wait for DRDY
        return false;
      }
    }
  }
  // RDY is now true, done waiting, success
  return true;
}

bool ADS1256::isDRDY()
{
  bool Ready = !digitalRead(ReadyPin); // rdy bit/pin is active low
  return Ready;
}

void ADS1256::pulsePowerDown()
{
  // a power down pulse must be at least t16
  // delay t16 is 4*Tclk, which is equal to t11
  digitalWrite(PowerDownPin, LOW);
  delayMicroseconds(DelayT11_4);
  digitalWrite(PowerDownPin, HIGH);
}
