
/*============================== (c) 2022 ADTP ================================
** File Name   :  NGL_FireBeetle_V1                                          **
** Author      :  Benoit                                                     **
** Created on  :  Jui 1, 2022                                                **
**---------------------------------------------------------------------------**
** Description : Controls the NGL_Proto_ST PCB                               **
*                 with ESP32 WROOM-32D-N16                                   **
=============================================================================*/



/*=============================================================================
**                                 PINOUT                                     **
=============================================================================*/

/*
------ SPI ------
MISO      --> IO12
MOSI      --> IO13
CLK       --> IO14

CS_SPARE  --> IO16
CS_R10    --> IO16
CS_R2     --> IO17

------ I²C ------
SCL       --> IO21
SDA       --> IO22

------ ADC ------
OUT_NTC   --> IO36 --> SENSOR_VP 
OUT_BAT   --> IO39 --> SENSOR_VN

------ OTHERS ------

LED BLUE  --> IO25

CLOCK_GEN --> IO32

INH       --> IO23

NEO       --> IO26

MOTEUR    --> IO18


Flash par UART ! 
ESP32 uniquement en 3.3 V ! 


Hardware:

Pont div sur VBAT: VS/VIN = 0.77
Full charge à 4.2 V

ADC MCP3426A0-E_SN
16 Bits _ 15 SPS

thermistor parameters:
 * RT0: 30 000 Ω
 * B: 3977 K +- 0.75%
 * T0:  25 C
 * +- 5%

INFOS:
Librairie ESP32 Utilisé avec comme board ESP32_FireBeetle pour l'ESP Wroom
https://techtutorialsx.com/2017/06/05/esp-wroom-32-uploading-a-program-with-arduino-ide/

BOARD: ESP32 Arduino --> ESP32 Dev Module
Partition size = 16 MB
*/

#include <Wire.h>
#include <MCP342x.h>
#include "BluetoothSerial.h"

#include "Config_NGL.h"
//#include <WiFi.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>




  /*==============================================================================
**                             Local Defines                                    **
================================================================================*/
#define LED_PIN      25  
#define CLOCK_PIN    32
#define MOTEUR_PIN   18
#define I2C_SDA_PIN  22
#define I2C_SCL_PIN  21
#define OUT_BAT_PIN  39
#define OUT_NTC_PIN  36

#define DELAY_PWM         10
#define DELAY_LOOP        420
#define DELAY_SEND_BLE    51

#define CLOCK_FREQ   1500


#define BRIGHTNESS_LED 121    // 8 Bits

#define VREF_ADC 2.048
//#define Addr 0x68

#define RT0 30000   // Ω
#define B 3977      // K
#define VCC 3.30    //Supply voltage
#define R 10000  //R=10KΩ

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

 /*===============================================================================
 **                            Global Variables                                 **
 ===============================================================================*/



// Le OUT BATTERIE est connecté au GPIO 36 (Pin VP)
//const int Out_Bat_PIN = 39;
//const int Out_Ntc_PIN = 36;

int OutBatValue = 0;
int OutNtcValue = 0;

float OutBatVolt = 0;
float fOutSens_V = 0;
float fOutDiff_V = 0;
float RT, VR, ln, TXX, Temp_0, VRT;

 int pwmChannel = 0; //Choisit le canal 0
 int pwmChannel_clock = 1; //Choisit le canal 0
 int resolution = 8; // Résolution de 8 bits, 256 valeurs possibles

uint8_t address = 0x68;
MCP342x adc = MCP342x(address);

BluetoothSerial SerialBT;

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
//uint32_t value = 0;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
 
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};
 


 /*===============================================================================
 **                            SETUP()                                          **
 ===============================================================================*/

void setup() {
/*
  //********* Hello woorld de test ********
// Set LED as output
    pinMode(LED, OUTPUT);

     */
  Temp_0 = 25 + 273.15;

  Setup_PWM();
  Setup_ADC();
  Setup_I2C();
  Setup_SERIAL();
//________________________BLE______________________
    // Create the BLE Device
  BLEDevice::init("ESP32-BLE_NGL-Proto");
 
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
 
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
 
  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
 
  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());
 
  // Start the service
  pService->start();
 
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");



}

 /*===============================================================================
 **                            LOOP()                                          **
 ===============================================================================*/
 
void loop() {
SerialBT.println("Bouillon en cours de mesure........");
/*
   //********* Hello woorld de test ********
    Serial.print("Hello");
    digitalWrite(LED, HIGH);
    
    delay(100);
    
    Serial.println(" world!");
    digitalWrite(LED, LOW);
    
    delay(100);
*/ 
 


         // ********* DIMMING LED *********
    for(int dutyCycle = 0; dutyCycle <= 169; dutyCycle++){
      
        ledcWrite(pwmChannel, dutyCycle);
        //ledcWrite(pwmChannel, dutyCycle);//1.65 V
        delay(DELAY_PWM);
                                                         }
    // ********* CLOCK *********
             ledcWrite(pwmChannel_clock, 127); //1.65 V de temps haut --> clock  


/*
       // ********* READING ADC *********
          // Tension batterie
        OutBatValue = analogRead(Out_Bat_PIN);
        OutBatVolt  = OutBatValue*(3.3/4096)/0.77; //Conversion en volt des 12 Bits
        Serial.print("Out_Bat = ");
        Serial.print(OutBatVolt);
        Serial.println(" V ");
        delay(250);
*/
          // ********* NTC *********
        VRT = analogRead(OUT_NTC_PIN );
        VRT = (3.30 / 4096.00) * VRT;      //Conversion to voltage
        Serial.print("Out_NTC = ");
        Serial.print(VRT);
        Serial.print(" V ");
        Serial.print(" \t ");
        
        VR = VCC - VRT;
        RT = VRT / (VR / R);               //Resistance of RT

        ln = log(RT / RT0);
        TXX = (1 / ((ln / B) + (1 / Temp_0))); //Temperature from thermistor
 
        TXX = TXX - 273.15;                 //Conversion to Celsius
 
        Serial.print("Temp = ");
        Serial.print(TXX);
        Serial.print(" °C ");
        Serial.print(" \t ");
        //BT
        SerialBT.print("Temp = ");
        SerialBT.print(TXX);
        SerialBT.print(" °C ");
        SerialBT.print(" \t ");

       
        delay(250);
        
       // ********* I²C *********  

//***** Channel 1
  long value = 0;  // Essai en int  16 Signed: int16_t
   //int16_t value = 0;
 
  MCP342x::Config status;
  // Initiate a conversion; convertAndRead() will wait until it can be read
  uint8_t err = adc.convertAndRead(MCP342x::channel1, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, value, status);
  if (err) {
    Serial.print("Convert error: ");
    Serial.println(err);
  }
  else {
    Serial.print("Out_SENS_16Bits_signed: ");
    Serial.print(value);
    Serial.print(" \t ");
  }
  
 

  //***** Channel 2

   long valueDiff = 0;
   // Essai en int  16 Signed: int16_t
   //int16_t valueDiff = 0;
//  MCP342x::Config status;
  // Initiate a conversion; convertAndRead() will wait until it can be read
  uint8_t err2 = adc.convertAndRead(MCP342x::channel2, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, valueDiff, status);
  if (err) {
    Serial.print("Convert error: ");
    Serial.println(err);
  }
  else {
    Serial.print("Out_DIFF_16Bits_signed: ");
    Serial.print(valueDiff);
    Serial.print(" \t ");
  }
  
 

// 32 767 est le max puisqu'il y a un bit de signe
// Donc sur 15 Bits puisqu'on est en single ended

 // ********* Conversion en Volt ********* 

    fOutSens_V = (VREF_ADC/32767 )* value; // Attention vref = 2.048 V
    Serial.print("OUT_SENS = ");
    Serial.print(fOutSens_V);
    Serial.print(" V ");
    Serial.print(" \t ");
    //BT
    SerialBT.print("OUT_SENS = ");
    SerialBT.print(fOutSens_V);
    SerialBT.println(" V "); //last BT
              
  
    fOutDiff_V = (VREF_ADC/32767 )* valueDiff;
    Serial.print("OUT_DIFF = ");
    Serial.print(fOutDiff_V);
    Serial.println(" V "); //last Serial

// ********* BT ********* 
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  delay(20);
//------------------ BLE -------------------------
  if (deviceConnected) {
  // Transformqtion en chaine de carac
        char txString[8];
        dtostrf(value, 1, 2, txString);

        pCharacteristic->setValue(txString); // SEND

        pCharacteristic->notify();
        //Serial.println(value);
        
        delay(DELAY_SEND_BLE); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
   
    delay(DELAY_LOOP) ;
    // Voir pour plus de chiffre sur le float
    
   
}       
         
