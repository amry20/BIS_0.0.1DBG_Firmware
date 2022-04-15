#include "Arduino.h"
#include "bis.h"
#include "gpio.h"
#include <SPI.h>
#include <MD_AD9833.h>
MD_AD9833	DDS_HIGH(AD9833_HIGH_FSYNC);  // Hardware SPI
MD_AD9833	DDS_LOW(AD9833_LOW_FSYNC);  // Hardware SPI
float_tag FloatValue;
void bis::init(){
    DDS_HIGH.begin();
    set_mode(0,HIGH_DDS);
    DDS_LOW.begin();
    set_mode(0,LOW_DDS);
    set_frequency(0);
    set_current(CurrentSet);
}
void bis::get_command(){
    if (Serial.available() > 0){
        CmdBuffer[SerialRxCount] = Serial.read();
        SerialRxCount++;
    }
    if (SerialRxCount >= CMD_LENGTH){
        SerialRxCount = 0;
        uint8_t crc = calculate_sum(CmdBuffer,CMD_LENGTH-1);
        if ((CmdBuffer[0] == START_MSG) && (CmdBuffer[CMD_LENGTH-1] == crc)){
            if (CmdBuffer[1] == Handshake){
                IdleStat = false;
                IdleStat = false;
                Connected = true;
                FirstSetup = true;
                digitalWriteFast(STATUS_LED,LOW);
                digitalWriteFast(ALERT_SOUND,LOW);
                delay(100);
                digitalWriteFast(ALERT_SOUND,HIGH);
                delay(100);
                digitalWriteFast(ALERT_SOUND,LOW);
                delay(100);
                digitalWriteFast(ALERT_SOUND,HIGH);
                /*Send reply after handshake message received*/
                uint8_t msg[MSG_LENGTH];
                memset(msg,0,MSG_LENGTH);
                msg[0] = START_MSG;
                msg[1] = Handshake;
                memcpy((char*)msg + 2,VersionTag,sizeof(VersionTag));
                msg[MSG_LENGTH-1] = calculate_sum(msg,MSG_LENGTH-1);
                send_msg(msg,MSG_LENGTH);
            }
            else if ((CmdBuffer[1] == SetFrequency)){
                /*Assembly 4 bytes to float (litle endian)*/
                FloatValue.b[0] = CmdBuffer[2];
                FloatValue.b[1] = CmdBuffer[3];
                FloatValue.b[2] = CmdBuffer[4];
                FloatValue.b[3] = CmdBuffer[5];
                FrequencySet = FloatValue.fval;
                set_frequency(FrequencySet);
                SampleCount = 0;
            }
            else if (CmdBuffer[1] == SetWaveForm){
                /*Waveform data on byte 2*/
                WaveForm = CmdBuffer[2];
                if (FrequencySet <= 1000)
                    set_mode(WaveForm, LOW_DDS);
                else set_mode(WaveForm, HIGH_DDS);
            }
            else if (CmdBuffer[1] == SelectCurrent){
                /*Current data on byte 2*/
                CurrentSet = CmdBuffer[2];
                set_current(CurrentSet);
            }
            else if (CmdBuffer[1] == SetPhase){
                /*Phase data on byte 2 and byte 3*/
                PhaseSet = CmdBuffer[2] + (CmdBuffer[3] << 8);
                set_phase(PhaseSet);
            }
            else if (CmdBuffer[1] == DoDisconnect){
                IdleStat = true;
                StartMeasurement = false;
                Connected = false;
                FirstSetup = true;
                digitalWriteFast(ALERT_SOUND,LOW);
                delay(100);
                digitalWriteFast(ALERT_SOUND,HIGH);
                delay(100);
            }
            else if (CmdBuffer[1] == Measurement){
                StartMeasurement = true;
                FirstSetup = true;
                digitalWriteFast(STATUS_LED,HIGH);
            }
            else if (CmdBuffer[1] == DoneMeasurement){
                StartMeasurement = false;
                SendGPDValue = false;
                digitalWriteFast(STATUS_LED,HIGH);
                SampleCount = 0;
            }
            else if (CmdBuffer[1] == GetGPDValue){
                SendGPDValue = true;
                SampleCount = 0;
            }
            else if (CmdBuffer[1] == StopGetGPD){
                SendGPDValue = false;
                SampleCount = 0;
            }
            else if (CmdBuffer[1] == GetCCVolt){
                send_ccvolt();
            }
            memset(CmdBuffer,0,CMD_LENGTH);
        }
    }
}

void bis::set_current(byte c){
    digitalWriteFast(MUX_EN,LOW);
    switch (c)
    {
    case C_5u:
        digitalWriteFast(MUX_A0, LOW);
        digitalWriteFast(MUX_A1, LOW);
        digitalWriteFast(MUX_A2, LOW);
        break;
    case C_10u:
        digitalWriteFast(MUX_A0, HIGH);
        digitalWriteFast(MUX_A1, LOW);
        digitalWriteFast(MUX_A2, LOW);
        break;
    case C_50u:
        digitalWriteFast(MUX_A0, LOW);
        digitalWriteFast(MUX_A1, HIGH);
        digitalWriteFast(MUX_A2, LOW);
        break;
    case C_100u:
        digitalWriteFast(MUX_A0, HIGH);
        digitalWriteFast(MUX_A1, HIGH);
        digitalWriteFast(MUX_A2, LOW);
        break;
    default:
        break;
    }

    digitalWriteFast(MUX_EN,HIGH);
}

void bis::set_frequency(float f){
    if (f <= 1000){
        set_mode(0,HIGH_DDS);
        DDS_LOW.setFrequency(MD_AD9833::CHAN_0,f);
    }
    else{ 
        set_mode(0,LOW_DDS);
        DDS_HIGH.setFrequency(MD_AD9833::CHAN_0,f);
    }
}

uint8_t bis::calculate_sum(uint8_t *bytes, int len){
  int i;
  uint8_t checksum = 0x00;
  for (i=0; i<len; i++)
      checksum +=bytes[i];
  checksum = (checksum^0xFF);
  checksum += checksum + 1;
  return checksum;
}

void bis::set_mode(uint8_t mode, uint8_t DDS){
    switch (mode){
    case 0:
        if (DDS == LOW_DDS)
            DDS_LOW.setMode(DDS_LOW.MODE_OFF);
        else
            DDS_HIGH.setMode(DDS_HIGH.MODE_OFF);
        break;
     case 1:
        if (DDS == LOW_DDS)
            DDS_LOW.setMode(DDS_LOW.MODE_SINE);
        else
            DDS_HIGH.setMode(DDS_HIGH.MODE_SINE);
        break;
     case 2:
        if (DDS == LOW_DDS)
            DDS_LOW.setMode(DDS_LOW.MODE_SQUARE1);
        else
        DDS_HIGH.setMode(DDS_HIGH.MODE_SQUARE1);
        break;
     case 3:
        if (DDS == LOW_DDS)
            DDS_LOW.setMode(DDS_LOW.MODE_SQUARE2);
        else
            DDS_HIGH.setMode(DDS_HIGH.MODE_SQUARE2);
        break;
     case 4:
        if (DDS == LOW_DDS)
            DDS_LOW.setMode(DDS_LOW.MODE_TRIANGLE);
        else
            DDS_HIGH.setMode(DDS_HIGH.MODE_TRIANGLE);
        break;
    default:
        if (DDS == LOW_DDS)
            DDS_LOW.setMode(DDS_LOW.MODE_SINE);
        else
            DDS_HIGH.setMode(DDS_HIGH.MODE_SINE);
        break;
    }
}
void bis::send_msg(const uint8_t *msg, uint8_t length){
    Serial.write(msg, length);
}

void bis::set_phase(uint16_t p){
    DDS_LOW.setPhase(DDS_LOW.CHAN_0,p);
}

void bis::send_gpd_value(uint16_t ADC_mag, uint16_t ADC_Phase, uint32_t SampleCount){
    uint8_t msg[MSG_LENGTH];
    memset(msg,0,MSG_LENGTH);
    msg[0] = START_MSG;
    msg[1] = GPDValue;
    msg[2] = (ADC_mag >> 8) & 0xFF;
    msg[3] = ADC_mag & 0xFF; 
    msg[4] = (ADC_Phase >> 8) & 0xFF;
    msg[5] = ADC_Phase & 0xFF; 
    msg[6] = (SampleCount >> 24) & 0xFF;
    msg[7] = (SampleCount >> 16) & 0xFF;
    msg[8] = (SampleCount >> 8) & 0xFF;
    msg[9] = SampleCount & 0xFF; 
    msg[MSG_LENGTH-1] = calculate_sum(msg,MSG_LENGTH-1);
    send_msg(msg,MSG_LENGTH);
}

void bis::send_ccvolt(){
    /*Deassembly 4 bytes to float (litle endian)*/
    uint8_t msg[MSG_LENGTH];
    memset(msg,0,MSG_LENGTH);
    msg[0] = START_MSG;
    msg[1] = GetCCVolt;
    FloatValue.fval = CCVolt1;
    msg[2] = FloatValue.b[0];
    msg[3] = FloatValue.b[1];
    msg[4] = FloatValue.b[2];
    msg[5] = FloatValue.b[3];
    FloatValue.fval = CCVolt2;
    msg[6] = FloatValue.b[0];
    msg[7] = FloatValue.b[1];
    msg[8] = FloatValue.b[2];
    msg[9] = FloatValue.b[3];
    msg[MSG_LENGTH-1] = calculate_sum(msg,MSG_LENGTH-1);
    send_msg(msg,MSG_LENGTH);
}

void bis::get_ccvolt(){
    CCVolt1 = (analogRead(PA1) / 4096 * 1.8)/0.6;
    CCVolt2 = (analogRead(PA2) / 4096 * 1.8)/0.6;
}