#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <driver/dac.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "defs.h"
#include "params.h"
#include "config.h"

bool InfoSend = 0;
bool ledStatus = 0;
bool CalibZeroSelected = 0;
bool CalibMaxSelected = 0;

CParameters params;
extern CParameters params;

const char* ssid = "ESP32 Metal Detector";
const char* password = "detectorupdate";

int VDI = 0;
int MSG = 0;

char DataString[16];

BLECharacteristic *pCharacteristic_RX;
BLECharacteristic *pCharacteristic_TX;

bool deviceConnected = false;

#define SERVICE_UUID                 "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      ledStatus = 0;
      InfoSend = 1;
      digitalWrite(2, HIGH);
      Serial.println("Bluetooth Connected");
    };

void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      ledStatus = 1;
      Serial.println("Bluetooth Disonnected");
      //ESP.restart();
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

//////////////////////////////////////////////////////////////////

if (rxValue.find("B") != -1)
        { 
     if (params.m_minSignal <255)
        {
         params.m_minSignal ++;//if pin0 is pressed and the duty ratio value is less than 255
         delay(50);                                               
        }
        }
     if (rxValue.find("A") != -1)
        {
     if (params.m_minSignal >0)
        {
         params.m_minSignal --;// if pin1 is pressed and the duty ratio value is greater than 0
         delay(50);                                              
        }                                                
}
        
//////////////////////////////////////////////////////////////////

if (rxValue.find("D") != -1)
        { 
     if (params.m_readingSamples <255)
        {
         params.m_readingSamples ++;//if pin0 is pressed and the duty ratio value is less than 255
         delay(50);                                               
        }
        }
     if (rxValue.find("C") != -1)
        {
     if (params.m_readingSamples >0)
        {
         params.m_readingSamples --;// if pin1 is pressed and the duty ratio value is greater than 0
         delay(50);                                              
        }                                                
}
         
//////////////////////////////////////////////////////////////////

if (rxValue.find("F") != -1)
        { 
     if (params.m_Phase_X <255)
        {
         params.m_Phase_X ++;//if pin0 is pressed and the duty ratio value is less than 255
         delay(50);                                               
        }
        }
     if (rxValue.find("E") != -1)
        {
     if (params.m_Phase_X >0)
        {
         params.m_Phase_X --;// if pin1 is pressed and the duty ratio value is greater than 0
         delay(50);                                              
        }                                                
}
         
//////////////////////////////////////////////////////////////////

         if (rxValue.find("I") != -1) {
         CalibZeroSelected = 1;      
       }
       
         if (rxValue.find("J") != -1) {
         CalibMaxSelected = 1;
       }
             
         EEPROMWriteInt(2, params.m_minSignal);
         EEPROMWriteInt(10, params.m_readingSamples);
         EEPROMWriteInt(14, params.m_Phase_X);
         Serial.print("MinSignal EEPROM Write :"); 
         Serial.println(params.m_minSignal);
         Serial.print("ReadingSamples EEPROM Write :"); 
         Serial.println(params.m_readingSamples);
         Serial.print("Phase_X EEPROM Write :"); 
         Serial.println(params.m_Phase_X);
    }
};

///////////////////////////////////////////////////////////////////////////////////////////////////////
//signal processing
bool generateReading(int* pPhase, int* pStrength, unsigned char* signalValues, bool* pbadReading) 
{
    #define SAMPLE_BUFFER_SIZE 350
    unsigned char strenth[SAMPLE_BUFFER_SIZE] = {0,};
    unsigned char angle[SAMPLE_BUFFER_SIZE] = {0,};
    int index = 0;

    bool enoughSamples = false;
    unsigned long startTime = millis();
    bool needToClearLastResult = true;
    bool retValue = true;

    while(1)
   {
     if (ledStatus == 1) {
     LED();
   }   
     if (InfoSend == 1) {
     delay(2000);
     sprintf(DataString, "%d,%d,%d,%d,%d,%d",-99,200, params.m_minSignal, params.m_readingSamples, params.m_Phase_X, params.m_Battery);    
     pCharacteristic_TX->setValue(DataString); 
     pCharacteristic_TX->notify();
     Serial.println(DataString);
     InfoSend = 0;
  }   
    //ArduinoOTA.handle();
    if (CalibZeroSelected == 1) {
    onCalibZeroSelected();   
    CalibZeroSelected = 0;
  }
 
    if (CalibMaxSelected == 1) {
    onCalibMaxSelected(); 
    CalibMaxSelected = 0;
  } 
      //sample signals
      float X = 0.0F;
      float R = 0.0F;
      const int SAMPLING_LOOP = params.m_readingSamples;
      for(int n(0);n<SAMPLING_LOOP;n++)
      {
        X += analogRead(Rx_XPin);
        R += analogRead(Rx_RPin);
      }
      X /= SAMPLING_LOOP;
      R /= SAMPLING_LOOP;

      //if opamp is saturated; break and report it
      if(X >= params.m_maxSignal || R >= params.m_maxSignal)
      {
        *pbadReading = true;
        Serial.println("Sinal MAX !");

        MSG = 201;
        sprintf(DataString, "%d,%d,%d,%d,%d,%d",VDI, MSG, params.m_minSignal, params.m_readingSamples, params.m_Phase_X, params.m_Battery);        
        pCharacteristic_TX->setValue(DataString); 
        pCharacteristic_TX->notify();
        break;
      }
      
      //bring to "virtual ADC zero"
      X = X - params.m_XBias;
      R = R - params.m_RBias;  

      //screen out samples using signal R polarity as set in params
      // 0 - use any polarity
      //-1 - use only negative R
      // 1 - use only positive R
      if( (params.m_sigPolarity<0 && R>0) || (params.m_sigPolarity>0 && R<0) )
      {
        X = 0.0F;
        R = 0.0F;
      }

      //phase in degrees
      int phase = ( (X == 0.0F && R == 0.0F) ? 0 : atan2(X, R) * 57.295F ); 
      if(phase < -90)
       phase = -90;
      else if(phase > 90) 
       phase = 90;
        
      angle[index]   = phase + 90;
      strenth[index] = int(sqrt(X*X+R*R));
      index++;

      //ring buffer; wrap around
      if(index >= SAMPLE_BUFFER_SIZE)
      {
        enoughSamples = true; 
        index = 0;
      }

      if(!enoughSamples)
        continue;

      //find peak signal
      unsigned char maxVal = 0;
      int maxIndex = -1;
      for(int n(0); n<SAMPLE_BUFFER_SIZE; n++)
      {
        if(strenth[n] > maxVal)
        {
          maxVal = strenth[n];
          maxIndex = n;               
        }
      }

      //is the peak strong enough
      if(maxVal < params.m_minSignal)
      {
        //no strong signal is present; its a good time to clear LCD if needed
        if(needToClearLastResult && (millis() - startTime) > 4000)
        {
          unsigned char empty[40]={0};    
          needToClearLastResult = false;

         VDI = -99;
         MSG = 200;
         sprintf(DataString, "%d,%d,%d,%d,%d,%d",VDI, MSG, params.m_minSignal, params.m_readingSamples, params.m_Phase_X, params.m_Battery);       
         pCharacteristic_TX->setValue(DataString); 
         pCharacteristic_TX->notify();
        }
        continue;
      }

      //where in the buffer is the peak
      int dist = index - maxIndex;
      if(dist < 0)
        dist = SAMPLE_BUFFER_SIZE + dist;

      //check if the peak is in the middle of the buffer
      if(dist != SAMPLE_BUFFER_SIZE/2)
        continue; 

      //////////////////////////////////////////////////////////////////////////////////////////////////////////
      //we have strong enough peak in the middle of sampling buffer
      //calculate the result and break the loop
      float angleAvg = 0.0F;
      float strengthAvg = 0.0F;
      int avgSamples = 0;
      const int reach = 1;        
      int n = maxIndex - reach;
      while(avgSamples < (reach*2 + 1))
      {
        if(n<0)
          n += SAMPLE_BUFFER_SIZE;
        else if(n>=SAMPLE_BUFFER_SIZE) 
          n %= SAMPLE_BUFFER_SIZE;  
            
        angleAvg    += (int(angle[n]) - 90);
        strengthAvg += strenth[n];  
        avgSamples++;
        n++;
      }

      *pPhase = angleAvg/avgSamples;
      *pStrength = strengthAvg/avgSamples; 
      
      if (deviceConnected) {
      int VDI = angleAvg/avgSamples; // This could be an actual sensor reading!

      MSG = 0;
      sprintf(DataString, "%d,%d,%d,%d,%d,%d",VDI, MSG, params.m_minSignal, params.m_readingSamples, params.m_Phase_X, params.m_Battery);        
      pCharacteristic_TX->setValue(DataString); 
      pCharacteristic_TX->notify();
      }
      
       Serial.println(*pPhase);

      //prepare data for the 40x8 graph      
      int k = index;
      const int inc = SAMPLE_BUFFER_SIZE/40;  
      int32_t i = 0;      
      while(i<40){
        if(index>=SAMPLE_BUFFER_SIZE) 
           index -= SAMPLE_BUFFER_SIZE;  
        
        signalValues[i] = float(strenth[index])/float(maxVal) * 8;

        index+=inc;
        i++;
      }
      //measurement completed; break the loop
       break;
    }
  return retValue;
}

////////////////////////////////////////////////////////////////////////////////////
// Standard arduino functions
void setup()
{
  Serial.begin(115200);

  //WiFi.mode(WIFI_AP);
  //WiFi.softAP(ssid, password);
  //WiFi.softAPConfig(IPAddress (192,168,1,1),IPAddress (192,168,1,1),IPAddress (255,255,255,0));
  //WiFi.setHostname("ESP32_Metal_Detector");

  //ArduinoOTA.setHostname("ESP32_Metal_Detector");  
  //ArduinoOTA.begin();

 // Create the BLE Device
  BLEDevice::init("ESP32 Metal Detector");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());


  BLEService *pService = pServer->createService(SERVICE_UUID);
  

  BLECharacteristic *pCharacteristic_RX = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );                  
 
  pCharacteristic_RX->setCallbacks(new MyCallbacks());


  pCharacteristic_TX = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_INDICATE|
                      BLECharacteristic::PROPERTY_NOTIFY 
                    );
 
  pCharacteristic_TX->addDescriptor(new BLE2902());

  pService->start();
  pServer->getAdvertising()->start();

  analogSetWidth(10); 
  //analogSetCycles(8);
  analogSetAttenuation(ADC_11db);
  //analogSetClockDiv(16);
  //analogSetSamples(16);

 /*
  analogReadResolution(12);             // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetWidth(12);                   // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
                                        //  9-bit gives an ADC range of 0-511
                                        // 10-bit gives an ADC range of 0-1023
                                        // 11-bit gives an ADC range of 0-2047
                                        // 12-bit gives an ADC range of 0-4095
  analogSetCycles(8);                   // Set number of cycles per sample, default is 8 and provides an optimal result, range is 1 - 255
  analogSetSamples(1);                  // Set number of samples in the range, default is 1, it has an effect on sensitivity has been multiplied
  analogSetClockDiv(1);                 // Set the divider for the ADC clock, default is 1, range is 1 - 255
  analogSetAttenuation(ADC_11db);       // Sets the input attenuation for ALL ADC inputs, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db
  analogSetPinAttenuation(VP,ADC_11db); // Sets the input attenuation, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db
                                        // ADC_0db provides no attenuation so IN/OUT = 1 / 1 an input of 3 volts remains at 3 volts before ADC measurement
                                        // ADC_2_5db provides an attenuation so that IN/OUT = 1 / 1.34 an input of 3 volts is reduced to 2.238 volts before ADC measurement
                                        // ADC_6db provides an attenuation so that IN/OUT = 1 / 2 an input of 3 volts is reduced to 1.500 volts before ADC measurement
                                        // ADC_11db provides an attenuation so that IN/OUT = 1 / 3.6 an input of 3 volts is reduced to 0.833 volts before ADC measurement
  adcAttachPin(VP);                     // Attach a pin to ADC (also clears any other analog mode that could be on), returns TRUE/FALSE result 
  adcStart(VP);                         // Starts an ADC conversion on attached pin's bus
  adcBusy(VP);                          // Check if conversion on the pin's ADC bus is currently running, returns TRUE/FALSE result 
  adcEnd(VP);                           // Get the result of the conversion (will wait if it have not finished), returns 16-bit integer result
  */

  pinMode(Rx_XPin, INPUT);
  pinMode(Rx_RPin, INPUT);
  pinMode(Tx_CoilPin, OUTPUT);
  pinMode(LedPin, OUTPUT);
  pinMode(VrefPin, OUTPUT);
  pinMode(BatPin, INPUT);
  pinMode(Phase_X_Pin, OUTPUT);
  pinMode(Phase_R_Pin, OUTPUT);
  digitalWrite(LedPin, LOW);
  
  EEPROM.begin(512);
  
  params.loadParams();
  //params.setDefaults();

  Battery();

  dacWrite(VrefPin, 127); 
   
  ledcAttachPin(Tx_CoilPin, 2);
  ledcSetup(2, params.m_CoilFrequency, 8); 
  ledcWriteTone(2, params.m_CoilFrequency); 
  delayMicroseconds(params.m_Phase_X);  
   
  ledcAttachPin(Phase_X_Pin, 4);
  ledcSetup(4, params.m_CoilFrequency, 8);
  ledcWriteTone(4, params.m_CoilFrequency);
  //ledcWrite(4, 100);
   
  delayMicroseconds(params.m_Phase_R);
  ledcAttachPin(Phase_R_Pin, 6);
  ledcSetup(6, params.m_CoilFrequency, 8);
  ledcWriteTone(6, params.m_CoilFrequency);
  
  Serial.println();
  Serial.println("Boot Finish Print Info");
  Serial.println();

  Serial.print("Hardware Version ");
  Serial.println(HARDWARE_VERSION);

  Serial.print("Firmware Version ");
  Serial.println(FIRMWARE_VERSION);

  Serial.print("WiFi SSID: ");
  Serial.println(ssid);

  Serial.print("WiFi Password: ");
  Serial.println(password);

  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  Serial.print("Battery: ");
  Serial.print(params.m_Battery);
  Serial.println("V");

  Serial.print("XBias ");
  Serial.println(params.m_XBias);

  Serial.print("RBias ");
  Serial.println(params.m_RBias); 

  Serial.print("minSignal ");
  Serial.println(params.m_minSignal);

  Serial.print("maxSignal ");
  Serial.println(params.m_maxSignal);

  Serial.print("readingSamples ");
  Serial.println(params.m_readingSamples);

  Serial.print("sigPolarity ");
  Serial.println(params.m_sigPolarity);

  Serial.print("Coil Frequency ");
  Serial.print(params.m_CoilFrequency);
  Serial.println(" Hz");

  Serial.print("Phase X ");
  Serial.println(params.m_Phase_X);

  Serial.print("Phase R ");
  Serial.println(params.m_Phase_R);

  ledStatus = 1;   
}

void loop()
{   
    int phase(0.0F);
    int strength(0);
    unsigned char signalValues[40]={0};    
    bool badReading(false);
    if(generateReading(&phase, &strength, signalValues, &badReading))
    {
      
    }
}
