#include "Arduino.h"
#include "bis.h"
#include "gpio.h"
#include <SPI.h>
#include <MD_AD9833.h>
MD_AD9833	DDS(FSYNC);  // Hardware SPI
float_tag FloatValue;
void bis::init(){
    DDS.begin();
    set_mode(0);
    set_frequency(1);
    DDS.setPhase(DDS.CHAN_0,PhaseSet);
    set_current(CurrentSet);
  
}
uint8_t bis::get_command(){
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
                return Handshake;
            }
            else if ((CmdBuffer[1] == SetFrequency)){
                /*Assembly 4 bytes to float (litle endian)*/
                FloatValue.b[0] = CmdBuffer[2];
                FloatValue.b[1] = CmdBuffer[3];
                FloatValue.b[2] = CmdBuffer[4];
                FloatValue.b[3] = CmdBuffer[5];
                FrequencySet = FloatValue.fval;
                return SetFrequency;
            }
            else if (CmdBuffer[1] == SetWaveForm){
                /*Waveform data on byte 2*/
                WaveForm = CmdBuffer[2];
                return SetWaveForm;
            }
            else if (CmdBuffer[1] == SelectCurrent){
                /*Current data on byte 2*/
                CurrentSet = CmdBuffer[2];
                return SelectCurrent;
            }
            else if (CmdBuffer[1] == SetPhase){
                /*Phase data on byte 2 and byte 3*/
                PhaseSet = CmdBuffer[2] + (CmdBuffer[3] << 8);
                return SetPhase;
            }
            else if (CmdBuffer[1] == DoDisconnect){
                return DoDisconnect;
            }
            else if (CmdBuffer[1] == SetSamplePoint){
                return SetSamplePoint;
            }
            else if (CmdBuffer[1] == Measurement){
                return Measurement;
            }
            else if (CmdBuffer[1] == DoneMeasurement){
                return DoneMeasurement;
            }
            else if (CmdBuffer[1] == GetGPDValue){
                return GetGPDValue;
            }
            else if (CmdBuffer[1] == StopGetGPD){
                return StopGetGPD;
            }
            memset(CmdBuffer,0,CMD_LENGTH);
        }
    }
    return IlegalOpcode;
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
    DDS.setFrequency(MD_AD9833::CHAN_0,f);
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

void bis::set_mode(uint8_t mode){
     switch (mode){
    case 0:
        DDS.setMode(DDS.MODE_OFF);
        break;
     case 1:
        DDS.setMode(DDS.MODE_SINE);
        break;
     case 2:
        DDS.setMode(DDS.MODE_SQUARE1);
        break;
     case 3:
        DDS.setMode(DDS.MODE_SQUARE2);
        break;
     case 4:
        DDS.setMode(DDS.MODE_TRIANGLE);
        break;
    default:
        DDS.setMode(DDS.MODE_SINE);
        break;
    }
}
void bis::send_msg(const uint8_t *msg, uint8_t length){
    Serial.write(msg, length);
}

void bis::set_phase(uint16_t p){
    DDS.setPhase(DDS.CHAN_0,p);
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