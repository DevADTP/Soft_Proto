
/*============================== (c) 2022 ADTP ================================
** File Name   :  NGL_FireBeetle_V1                                          **
** Author      :  Benoit                                                     **
** Created on  :  Jui 1, 2022                                                **
**---------------------------------------------------------------------------**
** Description : Controls the NGL_Proto_ST PCB                               **
                  with ESP32 WROOM-32D-N16                                   *
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

  LED BLUE      --> IO25

  CLOCK_GEN     --> IO32

  INH (EN_LDO)  --> IO23

  NEO           --> IO26

  MOTEUR        --> IO18


  Flash par UART !
  ESP32 uniquement en 3.3 V !


  Hardware:
  Pont div sur VBAT: VS/VIN = 0.77
  Full charge à 4.2 V

  ADC MCP3426A0-E_SN
  16 Bits _ 15 SPS

  thermistor parameters:
   RT0: 30 000 Ω
   B: 3977 K +- 0.75%
   T0:  25 C
   +- 5%

  INFOS:
  Librairie ESP32 Utilisé avec comme board ESP32_FireBeetle pour l'ESP Wroom
  https://techtutorialsx.com/2017/06/05/esp-wroom-32-uploading-a-program-with-arduino-ide/

  BOARD: ESP32 Arduino --> ESP32 Dev Module
  Partition size = 16 MB
*/

#include <Wire.h>
#include <MCP342x.h>
//#include "BluetoothSerial.h"

#include "Config_NGL.h"
//#include <WiFi.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>



/*==============================================================================
**                             Local Defines                                    **
  ================================================================================*/

#define EMULATEUR 0

#define LED_PIN      25
#define CLOCK_PIN    32
#define MOTEUR_PIN   18
#define I2C_SDA_PIN  22
#define I2C_SCL_PIN  21
#define OUT_BAT_PIN  39
#define OUT_NTC_PIN  36
#define INH_EN_LDO   23

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

// #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
// #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
// #endif

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

// #define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
// #define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// #define SERVICE_TEMP_UUID        "1defff16-5a0a-423b-b2d1-8abcddb67d8a"
// #define CHARACTERISTIC_TEMP_UUID "58f7494b-2e34-4bf3-ac42-9bd49445f277"

//original
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


//timing
#define WAIT_ACTIVE_ENABLE_POWER 10000
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

unsigned long ulong_time_now = 0;
unsigned long ulong_time_enablepower_change = 0;
int int_onetime_enable = 1;

float value2 = 0;
float value3_vib_cond = 0;

int pwmChannel = 0; //Choisit le canal 0
int pwmChannel_clock = 1; //Choisit le canal 0
int resolution = 8; // Résolution de 8 bits, 256 valeurs possibles

int int_ble_receive = 0;

uint8_t address = 0x68;
MCP342x adc = MCP342x(address);

//BluetoothSerial SerialBT;

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
//uint32_t value = 0;
long value = 0;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("BLE DEVICE CONNECTED    OK ");
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      Serial.println("BLE DEVICE DISCONNECTED   KO ");
      deviceConnected = false;
    }
};



class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received BLE: ");

        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
        }

        Serial.println();

        // Do stuff based on the command received from the app
        if (rxValue.find("1") != -1) {
          int_ble_receive = 1;
        }
        else if (rxValue.find("0") != -1) {
          int_ble_receive = 2;

        }

        Serial.println();
        Serial.println("*********");
      }
    }
};



/*===============================================================================
**                            SETUP()                                          **
  ===============================================================================*/

void setup() {

  //enable LDO
  pinMode(INH_EN_LDO, INPUT);
  //important DELAY FOR BOOTING
  delay(2000);

  Temp_0 = 25 + 273.15;

  //led
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  if (EMULATEUR == 0)
  {
    //Setup_IO(); //enable power
    Setup_PWM();
    Setup_ADC();
    Setup_I2C();
  }

  Setup_SERIAL();
  //________________________BLE______________________
  // Create the BLE Device
  BLEDevice::init("NGL Sensors");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE
                                          );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

  pAdvertising->addServiceUUID(SERVICE_UUID);

  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();

  Serial.println("Waiting a client connection to notify...");

  ulong_time_enablepower_change = millis() + WAIT_ACTIVE_ENABLE_POWER;

}

/*===============================================================================
**                            LOOP()                                          **
  ===============================================================================*/

void loop() {

  //period with millis()
  ulong_time_now = millis();

  if ((ulong_time_now >= ulong_time_enablepower_change))
  {
    ulong_time_enablepower_change = millis() + WAIT_ACTIVE_ENABLE_POWER;
    if (int_onetime_enable == 1)
    {
      int_onetime_enable = 0;
      Serial.println("ENABLE POWER activation autopower");
      Setup_IO(); //enable power
    }
  }


  if (EMULATEUR == 0)
  {
    //    // ********* DIMMING LED *********
    //    for (int dutyCycle = 0; dutyCycle <= 169; dutyCycle++) {
    //
    //      ledcWrite(pwmChannel, dutyCycle);
    //      //ledcWrite(pwmChannel, dutyCycle);//1.65 V
    //      delay(DELAY_PWM);
    //    }
    //    // ********* CLOCK *********
    //    ledcWrite(pwmChannel_clock, 127); //1.65 V de temps haut --> clock


    // ********* READING ADC *********
    // Tension batterie ADC esp32
    OutBatValue = analogRead(OUT_BAT_PIN);
    OutBatValue = analogRead(OUT_BAT_PIN);
    OutBatValue = analogRead(OUT_BAT_PIN);
    //ADC
    Serial.print(OutBatVolt);
    Serial.print(",");
    OutBatVolt  = OutBatValue * (3.3 / 4096) / 0.66; //Conversion en volt des 12 Bits
    //Serial.print("Out_Bat = ");
    //TENSION BAT 0.66*tension pile
    Serial.print(OutBatVolt);
    Serial.print(",");
    //delay(250);

    // ********* NTC *********
    //Mesure ADC NTC  ADC ESP32
    VRT = analogRead(OUT_NTC_PIN);
    VRT = analogRead(OUT_NTC_PIN);
    VRT = analogRead(OUT_NTC_PIN);

    //MESURE TENSION NTC
    //VRT = (3.30 / 4096.00) * VRT;      //Conversion to voltage
    //  Serial.print("Out_NTC = ");
    //  Serial.print(VRT);
    //  Serial.print(" V ");
    //  Serial.print(" \t ");
    //#define RT0 30000   // Ω
    //#define B 3977      // K
    //#define VCC 3.28    //Supply voltage
    //#define R 10000  //R=10KΩ

    VR = VCC - VRT;
    //RT = VRT / (VR / R);               //Resistance of RT

    //Rntc=Rpont*Vin/Vout-Rpont
    //RT = (VRT-VR)*10000/VR;               //Resistance of RT
    RT = (10000 * VCC / VR) - 10000;

    ln = log(RT / RT0);
    TXX = (1 / ((ln / B) + (1 / Temp_0))); //Temperature from thermistor

    TXX = TXX - 273.15;                 //Conversion to Celsius

    //  Serial.print("Temp = ");
    Serial.print(VRT);
    Serial.print(",");
    Serial.print(TXX);
    Serial.print(",");
    //  Serial.print(" °C ");
    //  Serial.print(" \t ");
    //BT
    //SerialBT.print("Temp = ");
    // SerialBT.print(TXX);
    // SerialBT.print(" °C ");
    // SerialBT.print(" \t ");

    //delay(250);

    // ********* I²C *********
    //***** Channel 1
    // OUT_SENS sortie chaine normale
    //long value = 0;  // Essai en int  16 Signed: int16_t
    //int16_t value = 0;

    MCP342x::Config status;
    // Initiate a conversion; convertAndRead() will wait until it can be read
    uint8_t err = adc.convertAndRead(MCP342x::channel1, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, value, status);
    value3_vib_cond = value;
    if (err) {
      Serial.print("ADC16bit OUT_SENS error: ");
      Serial.println(err);
    }
    else {
      //Serial.print("Out_SENS_16Bits_signed: ");
      Serial.print(value);
      Serial.print(",");
      Serial.print(value3_vib_cond);
      Serial.print(",");
      //Serial.print(" \t ");
    }



    //***** Channel 2
    //OUT_DIFF sortie ampli différentiel
    long valueDiff = 0;
    // Essai en int  16 Signed: int16_t
    //int16_t valueDiff = 0;
    //  MCP342x::Config status;
    // Initiate a conversion; convertAndRead() will wait until it can be read
    uint8_t err2 = adc.convertAndRead(MCP342x::channel2, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, valueDiff, status);
    if (err) {
      Serial.print("ADC16bit OUT_DIFF error: ");
      Serial.println(err);
    }
    else {
      //Serial.print("Out_DIFF_16Bits_signed: ");
      Serial.print(valueDiff);
      Serial.print(",");
      //Serial.print(" \t ");
    }



    // 32 767 est le max puisqu'il y a un bit de signe
    // Donc sur 15 Bits puisqu'on est en single ended

    // ********* Conversion en Volt *********

    fOutSens_V = (VREF_ADC / 32767 ) * value; // Attention vref = 2.048 V
    //Serial.print("OUT_SENS = ");
    Serial.println(fOutSens_V);
    //Serial.print(",");
    //Serial.print(" V ");
    //Serial.print(" \t ");
    //BT
    // SerialBT.print("OUT_SENS = ");
    // SerialBT.print(fOutSens_V);
    // SerialBT.println(" V "); //last BT

    //  fOutDiff_V = (VREF_ADC / 32767 ) * valueDiff;
    //  Serial.print("OUT_DIFF = ");
    //  Serial.print(fOutDiff_V);
    //  Serial.println(" V "); //last Serial

  }
  else
  {
    delay(200);
    //EMULATION
    //16bits
    value2 = value2 + 64;
    if (value2 >= 65536) value2 = 0;

    //12bits
    VRT = VRT + 4;
    if (VRT >= 4096) VRT = 0;

    Serial.print(value2);
    Serial.print(",");
    Serial.print(VRT);
    Serial.print(",");
  }

  // // ********* BT *********
  //   if (Serial.available()) {
  //     SerialBT.write(Serial.read());
  //   }
  //   if (SerialBT.available()) {
  //     Serial.write(SerialBT.read());
  //   }
  //   delay(20);
  //------------------ BLE -------------------------
  if (deviceConnected) {

    //blink led
    digitalWrite(LED_PIN, HIGH);
    delay(10);
    digitalWrite(LED_PIN, LOW);


    // // Transformation en chaine de carac
    //       char txString[8];
    //       dtostrf(value, 1, 2, txString);
    //       // char txStringTEMP[8];
    //       // dtostrf(VRT, 1, 2, txString);

    //       pCharacteristic->setValue(txString); // SEND
    //       // pCharacteristic_TEMP->setValue(txString); // SEND

    //       pCharacteristic->notify();
    //       // pCharacteristic_TEMP->notify();
    //       //Serial.println(value);

    //       delay(DELAY_SEND_BLE); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms

    // Let's convert the value to a char array:
    char txString[16]; // make sure this is big enuffz
    if (EMULATEUR == 0)
    {
      dtostrf(value3_vib_cond, 7, 0, txString); // float_val, min_width, digits_after_decimal, char_buffer
    }
    else
    {
      dtostrf(value2, 7, 0, txString); // float_val, min_width, digits_after_decimal, char_buffer
    }
    char txString2[10]; // make sure this is big enuffz
    dtostrf(VRT, 5, 1, txString2); // float_val, min_width, digits_after_decimal, char_buffer

    //    La fonction dtostrf() prend quatre paramètres d’entrée.
    //    La première est une variable de type double, que nous voulons convertir.
    //    La seconde est une variable de type char utilisée pour définir la largeur de la variable de sortie ou le nombre de chiffres.
    //    La troisième est une variable de type char utilisée pour définir le nombre de chiffres après la décimale.

    int indice = 8;
    txString[indice] = txString2[indice - 8];
    indice++;
    txString[indice] = txString2[indice - 8];
    indice++;
    txString[indice] = txString2[indice - 8];
    indice++;
    txString[indice] = txString2[indice - 8];
    indice++;
    txString[indice] = txString2[indice - 8];

    //txString[14] = char(10);
    txString[7] = 0x2C; //tab char(13);
    txString[15] = 0x0A; //return line char(13);

    //pTxCharacteristic->setValue(&txValue, 1);
    pCharacteristic->setValue(txString);
    //pTxCharacteristic->setValue(&txReturnValue, 8);
    // pTxCharacteristic->setValue(&txReturnValue2, 9);
    pCharacteristic->notify();
    delay(500);
    //if (int_type_measure == 1) txValue = 12 + random(0, 5);
    //if (int_type_measure == 2) txValue = 150 + random(0, 50);

    Serial.print("txstring : ");
    Serial.println(txString);
    
  }
  else

  {
    if (EMULATEUR == 1)
    {
      Serial.println();
    }
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
    Serial.println("Device connected ;-)");
  }

  //delay(DELAY_LOOP) ;
  if (!deviceConnected)
  {
    delay(2000);
    //blink led
    digitalWrite(LED_PIN, HIGH);
    delay(5);
    digitalWrite(LED_PIN, LOW);
  }
  
  // Voir pour plus de chiffre sur le float
}
