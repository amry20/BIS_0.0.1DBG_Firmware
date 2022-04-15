#include "Arduino.h"
#define VersionTag "BIS 1.0.0DBG"

// Pins for SPI comm with the AD9833 IC
#define DATA  PA_7	///< SPI Data pin number
#define CLK   PA_5	///< SPI Clock pin number

#define LOW_DDS 0x00
#define HIGH_DDS 0x01

#define CMD_LENGTH 7
#define MSG_LENGTH 19
#define START_MSG 0x02

#define C_100u 0x00
#define C_50u 0x01
#define C_10u 0x02
#define C_5u 0x03

#define V_MAG A1
#define V_PHASE A2

enum Opcode{
    IlegalOpcode = 0x00,
    Handshake = 0x01,
    SelectCurrent = 0x02,
    SetFrequency = 0x03,
    SetPhase = 0x04,
    SetWaveForm = 0x05,
    Measurement = 0x06,
    DoDisconnect = 0x07,
    SetSamplePoint = 0x08,
    GPDValue = 0x09,
    DoneMeasurement = 0xA,
    GetGPDValue = 0x0B,
    StopGetGPD = 0x0C,
    GetCCVolt = 0x0D
};
enum SamplePoint{
    Sample10 = 0x01,
    Sample20 = 0x02,
    Sample50 = 0x03,
    Sample100 = 0x04,
    Sample200 = 0x05
};
enum DAQStatus{
    Acquiring = 0x00,
    Finish = 0x01
};
union float_tag {
    byte b[4];
    float fval;
};
class bis {
    public:
        bool IdleStat = true;
        bool StartMeasurement = false;
        bool NewInjection = false;
        bool DoDAQ = false;
        bool SendGPDValue = false;
        bool Connected = false;
        bool FirstSetup = true;
        int LedCount = 0;
        float FrequencySet = 1000.0; /*Default is 1kHz */
        float CCVolt1, CCVolt2;
        uint8_t WaveForm =1; /*Defaule is sine wave (1)*/
        uint8_t SampleInterval = 10;
        uint8_t RecvCommand = IlegalOpcode;
        uint8_t CurrentSet = C_100u;
        uint8_t DAQStatus = Acquiring;
        uint8_t SamplePerSecond = Sample100;
        uint16_t PhaseSet = 0; /*Default phase is 0 degree*/
        uint16_t VMAG_ADC;
        uint16_t VPAHSE_ADC;
        uint16_t SampleNum = 0;
        uint32_t SampleCount = 0;
        void init();
        void get_command();
        void set_current(byte c);
        uint8_t calculate_sum(uint8_t *bytes, int len);
        void set_frequency(float f);
        void set_mode(uint8_t mode, uint8_t DDS);
        void send_msg(const uint8_t *msg, uint8_t length);
        void set_phase (uint16_t p);
        void send_gpd_value(uint16_t ADC_mag, uint16_t ADC_Phase, uint32_t SampleCount);
        void get_ccvolt();
    private:
        byte CmdBuffer[CMD_LENGTH];
        byte SerialRxCount = 0;
        void send_ccvolt();
};