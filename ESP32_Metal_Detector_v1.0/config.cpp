#include "defs.h"
#include "params.h"
#include "config.h"
#include <Arduino.h>
#include <EEPROM.h>

unsigned long   refreshTimer, flashTimer;

extern CParameters params;

void onCalibZeroSelected()
{
    Serial.println("Calibrating XBias,RBias");
    const int samples = 1000;
    unsigned long lX = 0;
    unsigned long lR = 0;    
    for(int s=0; s<samples; s++)
    {
        delay(1);   
        lX = lX + analogRead(Rx_XPin);
        lR = lR + analogRead(Rx_RPin);  
    }
    params.m_XBias = lX/float(samples);
    params.m_RBias = lR/float(samples);    

    EEPROMWriteInt(4, params.m_XBias);
    EEPROMWriteInt(6, params.m_RBias);

    Serial.print("m_XBias ");
    Serial.println(params.m_XBias);

    Serial.print("m_RBias ");
    Serial.println(params.m_RBias);

    Serial.println();
    Serial.println("Calibrating XBias,RBias Done");
    Serial.println();
    params.loadParams();
}

void onCalibMaxSelected()
{   
    Serial.println("maxSignal Calibrating");
    const int samples = 5000;
    int maxX = -1;
    int maxR = -1;    
    for(int s=0; s<samples; s++)
    {
        int X = analogRead(Rx_XPin);
        int R = analogRead(Rx_RPin);
        if(X>maxX) maxX = X;
        if(R>maxR) maxR = R;

        delay(1);         
    }

    int maxSignal = min(maxX, maxR) - 10;
    params.m_maxSignal = maxSignal;

    EEPROMWriteInt(8 , params.m_maxSignal); 

    Serial.print("maxSignal ");
    Serial.println(params.m_maxSignal);
    Serial.println();
    Serial.println("maxSignal Calibrating Done");
    params.loadParams();
}

void LED() {
  if (millis() - flashTimer < 50) { 
      digitalWrite(LedPin,HIGH);
      } 
      else if (millis() - flashTimer > 100) { 
      digitalWrite(LedPin,LOW);
      } 
      if (millis() - flashTimer > 6000) { 
      flashTimer = millis(); 
      }
}

void Battery() {
float VBAT = (350.0f/100.0f) * 3.30f * float(analogRead(BatPin)) / 1023.0f;  // LiPo battery
params.m_Battery = VBAT, 2;
delay(50);  
}
