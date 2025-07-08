/*
  ADS1256.h - Arduino Library for communication with Texas Instrument ADS1256 ADC
  Written by Adien Akhmad, August 2015
  Modfified  Jan 2019 by Axel Sepulveda for ATMEGA328
  Modified July 2025 by John Freudenthal to work with alternative SPI buses and connection settings.
*/

#include "ADS1256.h"
#include "Arduino.h"
#include "SPI.h"

ADS1256::ADS1256(SPIClass* spi, float clockspdMhz, float vref, uint8_t cspin, uint8_t readypin, uint8_t resetpin, uint8_t powerdownpin)
{
  SPIBus = spi;
  uint32_t SPISpeed = 1000000; //or clockspdMhz * 1000000 / 4 //May also want 5MHz at most
  ConnectionSettings = SPISettings(SPISpeed, MSBFIRST, SPI_MODE1);
  ReadyPin = readypin;
  ResetPin = resetpin;
  PowerDownPin = powerdownpin;
  CSPin = cspin;
  if (ReadyPin != ADS1256_NOPIN)
  {
    UseReady = true;
  }
  else
  {
    UseReady = false;
  }
  if (PowerDownPin != ADS1256_NOPIN)
  {
    UsePowerDown = true;
  }
  else
  {
    UsePowerDown = false;
  }
  if (ResetPin != ADS1256_NOPIN)
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
  SuppressCS = false;
  if (UseReady)
  {
    pinMode(ReadyPin, INPUT);
  }
  if (UseReset)
  {
    // set RESETPIN as output
    pinMode(ResetPin, OUTPUT);
    // pull RESETPIN high
    pinMode(ResetPin, HIGH);
  }
  pinMode(CSPin, OUTPUT);
  pinMode(CSPin, HIGH);
  if (UsePowerDown)
  {
    pinMode(PowerDownPin, OUTPUT);
    pinMode(PowerDownPin, HIGH);
  }
}

/*
Init chip with set datarate and gain and perform self calibration
*/ 
void ADS1256::begin(unsigned char drate, unsigned char gain, bool buffenable)
{
  _pga = 1 << gain;
  sendCommand(ADS1256_CMD_SDATAC); // send out ADS1256_CMD_SDATAC command to stop continous reading mode.
  writeRegister(ADS1256_RADD_DRATE, drate); // write data rate register
  uint8_t bytemask = B00000111;
  uint8_t adcon = readRegister(ADS1256_RADD_ADCON);
  uint8_t byte2send = (adcon & ~bytemask) | gain;
  bitWrite(byte2send,6,0);
  bitWrite(byte2send,5,0);
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

void ADS1256::startConversion()
{
  CSON(true);
  sendCommand(ADS1256_CMD_SYNC);
  sendCommand(ADS1256_CMD_WAKEUP);
  CSOFF(true);
}

float ADS1256::ReadAndSwitchChannels(byte channel)
{
  float Value;
  CSON(true);
  setChannel(channel);
  startConversion();
  Value = readCurrentChannel();
  CSOFF(true);
  return Value;
}

void ADS1256::writeRegister(unsigned char reg, unsigned char wdata)
{
  CSON();
  SPIBus->transfer(ADS1256_CMD_WREG | reg); // opcode1 Write registers starting from reg
  SPIBus->transfer(0); // opcode2 Write 1+0 registers
  SPIBus->transfer(wdata); // write wdata
  delayMicroseconds(DelayT11);
  CSOFF();
}

unsigned char ADS1256::readRegister(unsigned char reg)
{
  unsigned char readValue;
  CSON();
  SPIBus->transfer(ADS1256_CMD_RREG | reg); // opcode1 read registers starting from reg
  SPIBus->transfer(0); // opcode2 read 1+0 registers
  delayMicroseconds(DelayT6); // t6 delay (4*tCLKIN 50*0.13 = 6.5 us)
  readValue = SPIBus->transfer(0); // read registers
  delayMicroseconds(DelayT11); // t11 delay (4*tCLKIN 4*0.13 = 0.52 us)
  CSOFF();
  return readValue;
}

void ADS1256::PinMode(uint8_t Pin, uint8_t InOut)
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

bool ADS1256::DigitalRead(uint8_t Pin)
{
  if (Pin > 3)
  {
    Pin = 3;
  }
  uint8_t DigitalIO = readRegister(ADS1256_RADD_IO);
  DigitalIO = readRegister(ADS1256_RADD_IO);
  return bitRead(DigitalIO,Pin);
}

void ADS1256::DigitalWrite(uint8_t Pin, bool OffOn)
{
  if (Pin > 3)
  {
    Pin = 3;
  }
  uint8_t DigitalIO = readRegister(ADS1256_RADD_IO);
  bitWrite(DigitalIO, Pin, OffOn);
  writeRegister(ADS1256_RADD_IO, DigitalIO);
}

void ADS1256::sendCommand(unsigned char reg)
{
  CSON();
  if (UseReady)
  {
    waitDRDY();
  }
  SPIBus->transfer(reg);
  delayMicroseconds(DelayT11); // t11 delay (4*tCLKIN 4*0.13 = 0.52 us)
  CSOFF();
}

void ADS1256::setConversionFactor(float val)
{
  _conversionFactor = val;
}

void ADS1256::readTest()
{
  unsigned char _highByte, _midByte, _lowByte;
  CSON();
  SPIBus->transfer(ADS1256_CMD_RDATA);
  delayMicroseconds(DelayT6); // t6 delay (4*tCLKIN 50*0.13 = 6.5 us)
  _highByte = SPIBus->transfer(ADS1256_CMD_WAKEUP);
  _midByte = SPIBus->transfer(ADS1256_CMD_WAKEUP);
  _lowByte = SPIBus->transfer(ADS1256_CMD_WAKEUP);
  CSOFF();
  Serial.print("test:");
  Serial.print(_highByte);
  Serial.print(",");
  Serial.print(_midByte);
  Serial.print(",");
  Serial.print(_lowByte);
  Serial.print("\n");
}

float ADS1256::readCurrentChannel()
{
  CSON();
  SPIBus->transfer(ADS1256_CMD_RDATA);
  delayMicroseconds(DelayT6); // t6 delay (4*tCLKIN 50*0.13 = 6.5 us)
  float adsCode = read_float32();
  CSOFF();
  //Serial.print("absCode:");
  //Serial.print(adsCode);
  //Serial.print("\n");
  return ((adsCode / 0x7FFFFF) * ((2 * _VREF) / (float)_pga)) * _conversionFactor;
}

// Reads raw ADC data, as 32bit int
long ADS1256::readCurrentChannelRaw()
{
  CSON();
  SPIBus->transfer(ADS1256_CMD_RDATA);
  delayMicroseconds(DelayT6); // t6 delay (4*tCLKIN 50*0.13 = 6.5 us)
  long adsCode = read_int32();
  CSOFF();
  return adsCode;
}

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
  //Serial.print("bytes:");
  //Serial.print(_highByte);
  //Serial.print(",");
  //Serial.print(_midByte);
  //Serial.print(",");
  //Serial.print(_lowByte);
  //Serial.print("\n");
  return value;
}

// Call this ONLY after ADS1256_CMD_RDATA command
// Convert the signed 24bit stored in an unsigned 32bit to a signed 32bit
long ADS1256::read_int32()
{
  long value = read_uint24();
  //Serial.print("uint24:");
  //Serial.print(value);
  //Serial.print("\n");
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
  //Serial.print("int32v:");
  //Serial.print(value);
  //Serial.print("\n");
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
  //Serial.print("Start MUX set.\n");
  CSON(true);
  writeRegister(ADS1256_RADD_MUX, MUX_CHANNEL);
  //Serial.print("MUX set ");
  //Serial.print(AIN_P);
  //Serial.print(",");
  //Serial.print(AIN_N);
  //Serial.print("\n");
  sendCommand(ADS1256_CMD_SYNC);
  //Serial.print("SYNCED");
  sendCommand(ADS1256_CMD_WAKEUP);
  //Serial.print("WOKE");
  CSOFF(true);
  //Serial.print("CS off.\n");
}

/*
Reads and returns STATUS register
*/ 
uint8_t ADS1256::getStatus()
{
  sendCommand(ADS1256_CMD_SDATAC); // send out ADS1256_CMD_SDATAC command to stop continous reading mode.
  return readRegister(ADS1256_RADD_STATUS); 
}

SPISettings* ADS1256::GetSPISettings()
{
  return &ConnectionSettings;
}

void ADS1256::CSON(bool Supression)
{
  if (Supression)
  {
    if (!SuppressCS)
    {
      //Serial.print("CS start suppressed.");
      SPIBus->beginTransaction(ConnectionSettings);
      digitalWrite(CSPin, LOW);
      SuppressCS = true;
      //Serial.print("CS start complete.");
    }
  }
  else
  {
    if (!SuppressCS)
    {
      //Serial.print("CS start normal.");
      SPIBus->beginTransaction(ConnectionSettings);
      digitalWrite(CSPin, LOW);
      //Serial.print("CS start complete.");
    }
  }
}

void ADS1256::CSOFF(bool Supression)
{
  if (Supression)
  {
    if (SuppressCS)
    {
      //Serial.print("CS end suppressed.");
      digitalWrite(CSPin, HIGH);
      SPIBus->endTransaction();
      SuppressCS = false;
      //Serial.print("CS end complete.");
    }
  }
  else
  {
    if (!SuppressCS)
    {
      //Serial.print("CS start normal.");
      digitalWrite(CSPin, HIGH);
      SPIBus->endTransaction();
      //Serial.print("CS end complete.");
    }
  }
}

void ADS1256::CSON()
{
  CSON(false);
}

void ADS1256::CSOFF()
{
  CSOFF(false);
}

void ADS1256::waitDRDY()
{
  bool KeepWaiting = true;
  uint32_t WaitStart = micros();
  bool PinState = true;
  bool Ready = false;
  uint8_t Status;
  while (KeepWaiting)
  {
    if (UseReady)
    {
      PinState = digitalRead(ReadyPin);
      Ready = !PinState;
    }
    else
    {
      Status = getStatus();
      PinState = bitRead(Status,0);
      Ready = !PinState;
    }
    if (Ready)
    {
      KeepWaiting = false;
    }
    else
    {
      if ( (micros() - WaitStart) > 500000)
      {
        Serial.println("ADS1256 error on waitDRDY.");
        KeepWaiting = false;
      }
    }
  }
}

boolean ADS1256::isDRDY()
{
  bool PinState = true;
  bool Ready = false;
  uint8_t Status;
  if (UseReady)
  {
    PinState = digitalRead(ReadyPin);
    Ready = !PinState;
  }
  else
  {
    Status = getStatus();
    PinState = bitRead(Status,0);
    Ready = !PinState;
  }
  return Ready;
}
