#include <Arduino.h>
#include <IWatchdog.h>
#include "main.h"
#include "bis.h"
#include "gpio.h"
bis BisDAQ;
void setup() {
  IWatchdog.begin(5000000UL);
  IWatchdog.reload();
  Serial.begin(57600);
  init_gpio();
  BisDAQ.IdleStat = true;
  BisDAQ.init();
  analogReadResolution(12);
}

void loop() {
  // put your main code here, to run repeatedly:
  static bool SetFirst = false;
  static __ULong mTick = millis();
  heartbeat();
  BisDAQ.get_command();
  if (BisDAQ.IdleStat && !BisDAQ.Connected && !BisDAQ.StartMeasurement){
    idle_stat();
  }
  else if (BisDAQ.Connected && !BisDAQ.IdleStat && !BisDAQ.StartMeasurement){
    connected_stat();
  }
  else if (BisDAQ.Connected && BisDAQ.StartMeasurement && !BisDAQ.IdleStat){
    measurement_stat();
    if (mTick > millis()) mTick = 0;
    if (BisDAQ.SendGPDValue){
      if (millis() - mTick >= BisDAQ.SampleInterval){
        mTick = millis();
        BisDAQ.VMAG_ADC = analogRead(V_MAG);
        BisDAQ.VPAHSE_ADC =  analogRead(V_PHASE);
        BisDAQ.send_gpd_value(BisDAQ.VMAG_ADC, BisDAQ.VPAHSE_ADC, BisDAQ.SampleCount);
        BisDAQ.SampleCount++;
      }
    }
  }
  static __ULong CCReadInt = 0;
  if (CCReadInt > millis()) CCReadInt = 0;
  if (millis() - CCReadInt >= 10){
    CCReadInt = millis();
    BisDAQ.get_ccvolt();
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
  if (_mTick - Tick >= 500){
    Tick = _mTick;
    digitalToggleFast(STATUS_LED);
  }
}
void connected_stat(){
  __ULong _mTick = HAL_GetTick();
  static int ToggleCount = 0;
  static __ULong LedTime = 0;
  if (BisDAQ.FirstSetup){
    BisDAQ.FirstSetup = false;
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