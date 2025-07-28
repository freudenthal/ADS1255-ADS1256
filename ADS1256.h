/*
  ADS1256.h - Arduino Library for communication with Texas Instrument ADS1256 ADC
  Written by Adien Akhmad, August 2015
  Modifified  Jan 2019 by Axel Sepulveda for ATMEGA328
  Modified July 2025 by John Freudenthal to work with alternative SPI buses and connection settings.
*/

#ifndef ADS1256_h
#define ADS1256_h

// ADS1256 Register address
#define ADS1256_RADD_STATUS 0x00
#define ADS1256_RADD_MUX 0x01
#define ADS1256_RADD_ADCON 0x02
#define ADS1256_RADD_DRATE 0x03
#define ADS1256_RADD_IO 0x04
#define ADS1256_RADD_OFC0 0x05
#define ADS1256_RADD_OFC1 0x06
#define ADS1256_RADD_OFC2 0x07
#define ADS1256_RADD_FSC0 0x08
#define ADS1256_RADD_FSC1 0x09
#define ADS1256_RADD_FSC2 0x0A
// ADS1256 Command
#define ADS1256_CMD_WAKEUP 0x00
#define ADS1256_CMD_RDATA 0x01
#define ADS1256_CMD_RDATAC 0x03
#define ADS1256_CMD_SDATAC 0x0f
#define ADS1256_CMD_RREG 0x10
#define ADS1256_CMD_WREG 0x50
#define ADS1256_CMD_SELFCAL 0xF0
#define ADS1256_CMD_SELFOCAL 0xF1
#define ADS1256_CMD_SELFGCAL 0xF2
#define ADS1256_CMD_SYSOCAL 0xF3
#define ADS1256_CMD_SYSGCAL 0xF4
#define ADS1256_CMD_SYNC 0xFC
#define ADS1256_CMD_STANDBY 0xFD
#define ADS1256_CMD_RESET 0xFE
// define multiplexer positive codes
#define ADS1256_MUXP_AIN0 0x00
#define ADS1256_MUXP_AIN1 0x10
#define ADS1256_MUXP_AIN2 0x20
#define ADS1256_MUXP_AIN3 0x30
#define ADS1256_MUXP_AIN4 0x40
#define ADS1256_MUXP_AIN5 0x50
#define ADS1256_MUXP_AIN6 0x60
#define ADS1256_MUXP_AIN7 0x70
#define ADS1256_MUXP_AINCOM 0x80
// define multiplexer negative codes
#define ADS1256_MUXN_AIN0 0x00
#define ADS1256_MUXN_AIN1 0x01
#define ADS1256_MUXN_AIN2 0x02
#define ADS1256_MUXN_AIN3 0x03
#define ADS1256_MUXN_AIN4 0x04
#define ADS1256_MUXN_AIN5 0x05
#define ADS1256_MUXN_AIN6 0x06
#define ADS1256_MUXN_AIN7 0x07
#define ADS1256_MUXN_AINCOM 0x08
// define gain codes
#define ADS1256_GAIN_1 0x00
#define ADS1256_GAIN_2 0x01
#define ADS1256_GAIN_4 0x02
#define ADS1256_GAIN_8 0x03
#define ADS1256_GAIN_16 0x04
#define ADS1256_GAIN_32 0x05
#define ADS1256_GAIN_64 0x06
// define drate codes
/*
  NOTE : Data Rate vary depending on crystal frequency. Data rates listed below assumes the crystal frequency is 7.68Mhz for other frequency consult the datasheet.
*/
#define ADS1256_DRATE_30000SPS 0xF0
#define ADS1256_DRATE_15000SPS 0xE0
#define ADS1256_DRATE_7500SPS 0xD0
#define ADS1256_DRATE_3750SPS 0xC0
#define ADS1256_DRATE_2000SPS 0xB0
#define ADS1256_DRATE_1000SPS 0xA1
#define ADS1256_DRATE_500SPS 0x92
#define ADS1256_DRATE_100SPS 0x82
#define ADS1256_DRATE_60SPS 0x72
#define ADS1256_DRATE_50SPS 0x63
#define ADS1256_DRATE_30SPS 0x53
#define ADS1256_DRATE_25SPS 0x43
#define ADS1256_DRATE_15SPS 0x33
#define ADS1256_DRATE_10SPS 0x23
#define ADS1256_DRATE_5SPS 0x13
#define ADS1256_DRATE_2_5SPS 0x03
#define ADS1256_DRATE_NIL 0x00

#define ADS1256_NOPIN 255

#include "Arduino.h"
#include "SPI.h"

class ADS1256
{
    public:
        static constexpr uint8_t NUM_DATA_RATES = 16;
        static const uint8_t DATA_RATE_CODES[NUM_DATA_RATES];
        static const float DATA_RATES[NUM_DATA_RATES];
        static uint8_t dataRateCodeFromValue(float dataRate);
        static float dataRateValueFromCode(uint8_t dataRate);

        ADS1256(SPIClass* spi, float clockspdMhz, float vref, uint8_t cspin, uint8_t readypin, uint8_t resetpin, uint8_t powerdownpin);
        bool begin(unsigned char drate, unsigned char gain, bool bufferenable);
        
        
        float ReadAndSwitchChannels(byte channel);
        void setChannel(byte AIP, byte AIN = -1);
        
        uint8_t getStatus();
        void setDataRateCode(uint8_t dataRateCode);
        uint8_t getDataRateCode();
        float getDataRate();

        void GPIOPinMode(uint8_t Pin, uint8_t InOut);
        bool GPIOPinRead(uint8_t Pin);
        void GPIOPinWrite(uint8_t Pin, bool OffOn);
        
        void writeRegister(unsigned char reg, unsigned char wdata);
        unsigned char readRegister(unsigned char reg);
        
        void sync();
        bool standby();
        void wakeup();
        
        bool selfCalibrateAll();
        bool selfCalibrateGain();
        bool selfCalibrateOffset();
        bool systemCalibrateGain();
        bool systemCalibrateOffset();

        void pulsePowerDown();
        
        float readDataFloat();
        long readDataInt32();
        
        bool waitDRDY(uint32_t timeoutMicros = 500000);
        bool isDRDY();
        
        void setGain(uint8_t gain);
        SPISettings* GetSPISettings();
    private:
        SPIClass* SPIBus;
        SPISettings ConnectionSettings;
        bool CSEnabled;
        bool CSON();
        void CSOFF();
        unsigned long read_uint24();
        long read_int32();
        float read_float32();
        byte _pga;
        float _VREF;
        uint8_t ResetPin;
        uint8_t ReadyPin;
        uint8_t PowerDownPin;
        uint8_t CSPin;
        uint16_t DelayT11_4;
        uint16_t DelayT11_24;
        uint16_t DelayT6;
};

#endif
