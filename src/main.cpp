#include <Arduino.h>
#include <IWatchdog.h>
#include "main.h"
#include "bis.h"
#include "gpio.h"
bis BisDAQ;
bool FirstSetup = true;
void setup() {
  IWatchdog.begin(5000000UL);
  IWatchdog.reload();
  Serial.begin(57600);
  init_gpio();
  BisDAQ.IdleStat = true;
  BisDAQ.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  static bool SetFirst = false;
  static __ULong mTick = millis();
  heartbeat();
  BisDAQ.RecvCommand = BisDAQ.get_command();
  switch (BisDAQ.RecvCommand)
  {
    case IlegalOpcode:
      break;
    case Handshake:
      BisDAQ.IdleStat = false;
      BisDAQ.Connected = true;
      FirstSetup = true;
      break;
    case SetFrequency:
      BisDAQ.set_frequency(BisDAQ.FrequencySet);
      BisDAQ.SampleCount = 0;
      break;
    case SetWaveForm:
      BisDAQ.set_mode(BisDAQ.WaveForm);
      break;
    case SelectCurrent:
      BisDAQ.set_current(BisDAQ.CurrentSet);
      break;
    case SetPhase:
      BisDAQ.set_phase(BisDAQ.PhaseSet);
      break;
    case DoDisconnect:
      BisDAQ.IdleStat = true;
      BisDAQ.StartMeasurement = false;
      BisDAQ.Connected = false;
      FirstSetup = true;
      digitalWriteFast(ALERT_SOUND,LOW);
      delay(100);
      digitalWriteFast(ALERT_SOUND,HIGH);
      delay(100);
      break;
    case SetSamplePoint:
      if (BisDAQ.SamplePerSecond == Sample10){
        BisDAQ.SampleNum = 10; BisDAQ.SampleInterval = 100; //100ms
      }
      else if (BisDAQ.SamplePerSecond == Sample20){
        BisDAQ.SampleNum = 20; BisDAQ.SampleInterval = 50; //50ms
      }
      else if (BisDAQ.SamplePerSecond == Sample50){
        BisDAQ.SampleNum = 50; BisDAQ.SampleInterval = 20; //20ms
      }
      else if (BisDAQ.SamplePerSecond == Sample100){
        BisDAQ.SampleNum = 100; BisDAQ.SampleInterval = 10; //10ms
      }
      else if (BisDAQ.SamplePerSecond == Sample200){
        BisDAQ.SampleNum = 200; BisDAQ.SampleInterval = 5; //5ms
      }
      break;
    case Measurement:
      BisDAQ.StartMeasurement = true;
      digitalWriteFast(STATUS_LED,HIGH);
      break;
    case DoneMeasurement:
      BisDAQ.StartMeasurement = false;
      BisDAQ.SendGPDValue = false;
      digitalWriteFast(STATUS_LED,HIGH);
      BisDAQ.SampleCount = 0;
      break;
    case GetGPDValue:
      BisDAQ.SendGPDValue = true;
      break;
    case StopGetGPD:
      BisDAQ.SendGPDValue = false;
      break;
    default:
      break;
  }
  if (BisDAQ.IdleStat && !BisDAQ.Connected && !BisDAQ.StartMeasurement){
    idle_stat();
  }
  else if (BisDAQ.Connected && !BisDAQ.IdleStat && !BisDAQ.StartMeasurement){
    connected_stat();
  }
  else if (BisDAQ.Connected && BisDAQ.StartMeasurement && !BisDAQ.IdleStat){
    measurement_stat();
    if (BisDAQ.SendGPDValue){
      if (millis() - mTick >= BisDAQ.SampleInterval){
        mTick = millis();
        BisDAQ.VMAG_ADC = analogRead(V_MAG);
        BisDAQ.VPAHSE_ADC =  analogRead(V_PHASE);
        BisDAQ.send_gpd_value(BisDAQ.VMAG_ADC, BisDAQ.VPAHSE_ADC, BisDAQ.SampleCount);
        BisDAQ.SampleCount++;
      }
      if (mTick > millis()) mTick = 0;
    }
  }
}

void init_gpio(){
  pinMode(HEARTBEAT_LED,OUTPUT);
  pinMode(STATUS_LED,OUTPUT);
  pinMode(ALERT_SOUND,OUTPUT);
  pinMode(MUX_A0,OUTPUT);
  pinMode(MUX_A1,OUTPUT);
  pinMode(MUX_A2,OUTPUT);
  pinMode(MUX_EN,OUTPUT);
  digitalWriteFast(HEARTBEAT_LED,LOW);
  digitalWriteFast(STATUS_LED,LOW);
  digitalWriteFast(ALERT_SOUND,HIGH);
  digitalWriteFast(MUX_A0,LOW);
  digitalWriteFast(MUX_A1,LOW);
  digitalWriteFast(MUX_A2,LOW);
  digitalWriteFast(MUX_EN,HIGH);

  digitalWriteFast(ALERT_SOUND,LOW);
  delay(250);
  digitalWriteFast(ALERT_SOUND,HIGH);
  analogReadResolution(12);
}

void heartbeat(){
  static __ULong Tick = 0;
  __ULong _mTick = HAL_GetTick();
  if (_mTick - Tick >= 1000){
    Tick = _mTick;
    IWatchdog.reload();
    digitalToggleFast(HEARTBEAT_LED);
  }
}

void idle_stat(){
  static __ULong Tick = 0;
  __ULong _mTick = HAL_GetTick();
  if (Tick > _mTick)Tick = 0;
  if (_mTick - Tick >= 300){
    Tick = _mTick;
    if (BisDAQ.LedCount == 0){
      digitalWriteFast(STATUS_LED,LOW);
    }
    else if (BisDAQ.LedCount == 1){
      digitalWriteFast(STATUS_LED,HIGH);
      BisDAQ.LedCount = -1;
    }
    BisDAQ.LedCount++;
  }
}
void connected_stat(){
  __ULong _mTick = HAL_GetTick();
  static int ToggleCount = 0;
  static __ULong LedTime = 0;
  if (FirstSetup){
    FirstSetup = false;
    ToggleCount = 0;
    digitalWriteFast(STATUS_LED,HIGH);
  }
  if (LedTime > _mTick) LedTime = 0;
  if (_mTick - LedTime >= 250){
    LedTime = _mTick;
    if (ToggleCount < 4){
      digitalToggleFast(STATUS_LED);
    }
    else{
      digitalWriteFast(STATUS_LED,HIGH);
      if (ToggleCount >=7){
        ToggleCount = -1;
      }
    }
    ToggleCount++;
  }
}
void measurement_stat(){
    __ULong _mTick = HAL_GetTick();
  static __ULong LedTime = 0;
  if (LedTime > _mTick)LedTime = 0;
  if (_mTick - LedTime >=100){
    LedTime = _mTick;
    digitalToggleFast(STATUS_LED);
  }
}