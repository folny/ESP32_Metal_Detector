#include <Arduino.h>
#include <EEPROM.h>
#include "params.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////
//This function will write a 2 byte integer to the eeprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value)
{
    unsigned char lowByte = ((p_value >> 0) & 0xFF);
    unsigned char highByte = ((p_value >> 8) & 0xFF);

    EEPROM.write(p_address, lowByte);
    EEPROM.write(p_address + 1, highByte);
    EEPROM.commit();
}

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address)
{
    unsigned char lowByte = EEPROM.read(p_address);
    unsigned char highByte = EEPROM.read(p_address + 1);
    return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
CParameters::CParameters()

{
  setDefaults();
}

void CParameters::setDefaults()
{
      m_minSignal        = 20;
      m_XBias            = 512;
      m_RBias            = 512;
      m_maxSignal        = 2048;
      m_readingSamples   = 20;       
      m_sigPolarity      = 1;
      m_Phase_X          = 26; 
      m_Phase_R          = 109;
      m_CoilFrequency    = 8333;
      m_Battery          = 0;

      EEPROMWriteInt(2, m_minSignal);
      EEPROMWriteInt(4, m_XBias);
      EEPROMWriteInt(6, m_RBias);
      EEPROMWriteInt(8, m_maxSignal);
      EEPROMWriteInt(10, m_readingSamples);                   
      EEPROMWriteInt(12, m_sigPolarity);        
      EEPROMWriteInt(14, m_Phase_X);
      EEPROMWriteInt(16, m_Phase_R);
      EEPROMWriteInt(18, m_CoilFrequency);
      EEPROMWriteInt(20, m_Battery);
      EEPROM.commit();
};

void CParameters::loadParams()
{
      m_minSignal      = EEPROMReadInt(2);
      m_XBias          = EEPROMReadInt(4);
      m_RBias          = EEPROMReadInt(6);
      m_maxSignal      = EEPROMReadInt(8);
      m_readingSamples = EEPROMReadInt(10);
      m_sigPolarity    = EEPROMReadInt(12);
      m_Phase_X        = EEPROMReadInt(14);
      m_Phase_R        = EEPROMReadInt(16);
      m_CoilFrequency  = EEPROMReadInt(18);
      m_Battery        = EEPROMReadInt(20);
}

