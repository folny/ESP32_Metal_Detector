#ifndef MD_PARAMS_H
#define MD_PARAMS_H

class CParameters
{
public:
	CParameters();
	void setDefaults();
	void loadParams();


public:
	int m_minSignal; 		    //eeprom loc 2
	int m_XBias; 			      //eeprom loc 4
	int m_RBias; 			      //eeprom loc 6
  int m_maxSignal;		    //eeprom loc 8
	int m_readingSamples;  	//eeprom loc 10
  int m_sigPolarity;    	//eeprom loc 12
  int m_Phase_X;          //eeprom loc 14
  int m_Phase_R;          //eeprom loc 16
  int m_CoilFrequency;    //eeprom loc 18
  int m_Battery;          //eeprom loc 20
};

//
void EEPROMWriteInt(int p_address, int p_value);
unsigned int EEPROMReadInt(int p_address);

#endif

