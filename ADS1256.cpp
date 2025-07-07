/*
  ADS1256.h - Arduino Library for communication with Texas Instrument ADS1256 ADC
  Written by Adien Akhmad, August 2015
  Modfified  Jan 2019 by Axel Sepulveda for ATMEGA328
  Modified July 2025 by John Freudenthal to work with alternative SPI buses and connection settings.
*/

#include "ADS1256.h"
#include "Arduino.h"
#include "SPI.h"

ADS1256::ADS1256(SPIClass* spi, float clockspdMhz, float vref, uint8_t cspin, uint8_t readypin = 254, uint8_t resetpin = 254)
{
  SPIBus = spi;
  uint32_t SPISpeed = 1000000; //or clockspdMhz * 1000000 / 4 //May also want 5MHz at most
  ConnectionSettings = SPISettings(SPISpeed, MSBFIRST, SPI_MODE1);
  ReadyPin = readypin;
  ResetPin = resetpin;
  CSPin = cspin;
  if (ReadyPin != 254)
  {
    UseReady = true;
  }
  else
  {
    UseReady = false;
  }
  if (ResetPin != 254)
  {
    UseReset = true;
  }
  else
  {
    UseReset = false;
  }
  // Voltage Reference
  _VREF = vref;
  // Default conversion factor
  _conversionFactor = 1.0;
  DelayT6 = (uint16_t)((1.0/(float)(clockspdMhz)) * 50.0) + 1;
  DelayT11 = (uint16_t)((1.0/(float)(clockspdMhz)) * 4.0) + 1;
}

void ADS1256::writeRegister(unsigned char reg, unsigned char wdata)
{
  SPIBus->beginTransaction(ConnectionSettings);
  CSON();
  SPIBus->transfer(ADS1256_CMD_WREG | reg); // opcode1 Write registers starting from reg
  SPIBus->transfer(0); // opcode2 Write 1+0 registers
  SPIBus->transfer(wdata); // write wdata
  delayMicroseconds(DelayT11);
  SPIBus->endTransaction();
  CSOFF();
}

unsigned char ADS1256::readRegister(unsigned char reg)
{
  unsigned char readValue;
  SPIBus->beginTransaction(ConnectionSettings);
  CSON();
  SPIBus->transfer(ADS1256_CMD_RREG | reg); // opcode1 read registers starting from reg
  SPIBus->transfer(0); // opcode2 read 1+0 registers
  delayMicroseconds(DelayT6); // t6 delay (4*tCLKIN 50*0.13 = 6.5 us)
  readValue = SPIBus->transfer(0); // read registers
  delayMicroseconds(DelayT11); // t11 delay (4*tCLKIN 4*0.13 = 0.52 us)
  CSOFF();
  SPIBus->endTransaction();
  return readValue;
}

void ADS1256::sendCommand(unsigned char reg)
{
  SPIBus->beginTransaction(ConnectionSettings);
  CSON();
  waitDRDY();
  SPIBus->transfer(reg);
  delayMicroseconds(DelayT11); // t11 delay (4*tCLKIN 4*0.13 = 0.52 us)
  CSOFF();
  SPIBus->endTransaction();
}

void ADS1256::setConversionFactor(float val)
{
  _conversionFactor = val;
}

void ADS1256::readTest()
{
  unsigned char _highByte, _midByte, _lowByte;
  SPIBus->beginTransaction(ConnectionSettings);
  CSON();
  SPIBus->transfer(ADS1256_CMD_RDATA);
  delayMicroseconds(DelayT6); // t6 delay (4*tCLKIN 50*0.13 = 6.5 us)
  _highByte = SPIBus->transfer(ADS1256_CMD_WAKEUP);
  _midByte = SPIBus->transfer(ADS1256_CMD_WAKEUP);
  _lowByte = SPIBus->transfer(ADS1256_CMD_WAKEUP);
  CSOFF();
  SPIBus->endTransaction();
}

float ADS1256::readCurrentChannel()
{
  SPIBus->beginTransaction(ConnectionSettings);
  CSON();
  SPIBus->transfer(ADS1256_CMD_RDATA);
  delayMicroseconds(DelayT6); // t6 delay (4*tCLKIN 50*0.13 = 6.5 us)
  float adsCode = read_float32();
  CSOFF();
  SPIBus->endTransaction();
  return ((adsCode / 0x7FFFFF) * ((2 * _VREF) / (float)_pga)) * _conversionFactor;
}

// Reads raw ADC data, as 32bit int
long ADS1256::readCurrentChannelRaw()
{
  SPIBus->beginTransaction(ConnectionSettings);
  CSON();
  SPIBus->transfer(ADS1256_CMD_RDATA);
  delayMicroseconds(DelayT6); // t6 delay (4*tCLKIN 50*0.13 = 6.5 us)
  long adsCode = read_int32();
  CSOFF();
  SPIBus->endTransaction();
  return adsCode;
}

// Call this ONLY after ADS1256_CMD_RDATA command
unsigned long ADS1256::read_uint24()
{
  unsigned char _highByte, _midByte, _lowByte;
  unsigned long value;
  _highByte = SPI.transfer(0);
  _midByte  = SPI.transfer(0);
  _lowByte  = SPI.transfer(0);
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

// Call this ONLY after ADS1256_CMD_RDATA command
// Cast as a float
float ADS1256::read_float32()
{
  long value = read_int32();
  return (float)value;
}

// Channel switching for single ended mode. Negative input channel are automatically set to AINCOM
void ADS1256::setChannel(byte channel)
{
  setChannel(channel, -1);
}

// Channel Switching for differential mode. Use -1 to set input channel to AINCOM
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
  CSON();
  writeRegister(ADS1256_RADD_MUX, MUX_CHANNEL);
  sendCommand(ADS1256_CMD_SYNC);
  sendCommand(ADS1256_CMD_WAKEUP);
  CSOFF();
}

/*
Init chip with set datarate and gain and perform self calibration
*/ 
void ADS1256::begin(unsigned char drate, unsigned char gain, bool buffenable)
{
  if (UseReady)
  {
    pinMode(pinDRDY, INPUT);
  }
  if (UseReset)
  {
    // set RESETPIN as output
    pinMode(ResetPin, OUTPUT);
    // pull RESETPIN high
    pinMode(pinRST, HIGH);
  }
  pinMode(CSPin, OUTPUT);
  pinMode(CSPin, HIGH);
  _pga = 1 << gain;
  sendCommand(ADS1256_CMD_SDATAC); // send out ADS1256_CMD_SDATAC command to stop continous reading mode.
  writeRegister(ADS1256_RADD_DRATE, drate); // write data rate register
  uint8_t bytemask = B00000111;
  uint8_t adcon = readRegister(ADS1256_RADD_ADCON);
  uint8_t byte2send = (adcon & ~bytemask) | gain;
  writeRegister(ADS1256_RADD_ADCON, byte2send);
  if (buffenable)
  {  
    uint8_t status = readRegister(ADS1256_RADD_STATUS);
    bitSet(status, 1); 
    writeRegister(ADS1256_RADD_STATUS, status);
  }
  sendCommand(ADS1256_CMD_SELFCAL); // perform self calibration
  if (UseReady)
  {
    waitDRDY();
  }
}

/*
Reads and returns STATUS register
*/ 
uint8_t ADS1256::getStatus()
{
  sendCommand(ADS1256_CMD_SDATAC); // send out ADS1256_CMD_SDATAC command to stop continous reading mode.
  return readRegister(ADS1256_RADD_STATUS); 
}

SPISettings* ADS1263::GetSPISettings()
{
  return &ConnectionSettings;
}

void ADS1256::CSON()
{
  digitalWrite(CSPin, LOW);
}

void ADS1256::CSOFF()
{
  digitalWrite(CSPin, HIGH);
}

void ADS1256::waitDRDY()
{
  bool KeepWaiting = true;
  uint32_t WaitStart = micros();
  bool PinState = true;
  while (KeepWaiting)
  {
      PinState = digitalRead(pinDRDY);
      if (!PinState)
      {
        KeepWaiting = false;
      }
      else
      {
        if ( (micros() - WaitStart) > 1000000)
        {
          Serial.println("ADS1256 error on waitDRDY.");
          KeepWaiting = false;
        }
      }
  }
}

boolean ADS1256::isDRDY()
{
  return !digitalRead(pinDRDY);
}	
