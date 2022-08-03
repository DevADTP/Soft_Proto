/*============================== (c) 2022 ADTP ================================
** File Name   :  NGL_FireBeetle_V1                                          **
** Author      :  Benoit                                                     **
** Created on  :  Jui 1, 2022                                                **
** Modified    :  Sully
** Last        :  Jui 27, 2022
**---------------------------------------------------------------------------**
** Description : Controls the NGL_Proto_ST PCB                               **
                  with ESP32 WROOM-32D-N16                                   *
  =============================================================================*/



/*=============================================================================
**                                 PINOUT                                     **
  =============================================================================*/

/*
  EMULATEUR
  M5STACK  ATOM LITE connexion
                ------------
  GYRO -- 3V3  | 3V3        |
  GYRO <- CS   | G22    G21 |
  GYRO <- MOSI | G19    G25 |
  GYRO <- CLK  | G23    5V  |
  GYRO -> MISO | G33    GND | -- GYRO
                ------------



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

  AD5270 20k potentiometer:
  SPI (MOSI/MISO/CLK)
  CC: pin 16

  MAX5481 10k potentiometer:
  SPI (MOSI/CLK)
  CC: pin 17

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

#include <SPI.h>
#include "AD5270.h"



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
#define DELAY_LOOP_BLE_ACTIVE  500
#define DELAY_LOOP_BLE_NOTACTIVE  2000

#define CLOCK_FREQ   1500

#define BRIGHTNESS_LED 121    // 8 Bits

#define VREF_ADC 2.048
//#define Addr 0x68

#define RT0 30000   // Ω
#define B 3977      // K
#define VCC 3.30    //Supply voltage
#define R 10000  //R=10KΩ

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

// #define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
// #define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// #define SERVICE_TEMP_UUID        "1defff16-5a0a-423b-b2d1-8abcddb67d8a"
// #define CHARACTERISTIC_TEMP_UUID "58f7494b-2e34-4bf3-ac42-9bd49445f277"

//original NORDIC UUID
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


//timing
#define WAIT_ACTIVE_ENABLE_POWER 10000


//SPI POTENTIOMETER
//20K AD5270 (spi IN/OUT)
// carte NGL
#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_SCLK 14
#define HSPI_SS_AD5270_20K 16
#define HSPI_SS_MAX5481_10K 17 //origine carte 10K
#define HSPI_SS_AD5270_20K_IN 15  //modif ajout POT 20k externe CS_SPARE

//carte ELUMATEUR
//                ------------
//  GYRO -- 3V3  | 3V3        |
//  GYRO <- CS   | G22    G21 |
//  GYRO <- MOSI | G19    G25 |
//  GYRO <- CLK  | G23    5V  |
//  GYRO -> MISO | G33    GND | -- GYRO
//                ------------
//#define HSPI_MISO 33
//#define HSPI_MOSI 19
//#define HSPI_SCLK 23
//#define HSPI_SS_AD5270_20K 22
//#define HSPI_SS_MAX5481_10K 22

static const int spiClk = 500000;  // 1 MHz
int val_pot_20k = 0;
int val_pot_10k = 0;
char buffer_pot[30];
int virgCharIndex = 0;
char txStringSendBle[35];  //65536,65536,50.0,4095,20000,10060
int int_pot20k_input = 0;

SPIClass* hspi = NULL;
SPIClass* vspi = NULL;



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
long valueDiff = 0;

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
bool boolReceive = false;

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


String inStringBleReceive = "";
String StringRes20k = "";
String StringRes10k = "";
float floatRealValueRIn = 0.0;
float floatRealValueROut = 0.0;
  float RreadADC = 0.0;
  float floatvallocaltemp = 0.0;

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0)
      {
        boolReceive = true;
        for (int i = 0; i < rxValue.length(); i++)
        {
          Serial.print(rxValue[i]);
          inStringBleReceive += (char)rxValue[i];
        }
      }
    }
};



/*===============================================================================
**                            SETUP()                                          **
  ===============================================================================*/
/*
    _________       __
   /   _____/ _____/  |_ __ ________
   \_____  \_/ __ \   __\  |  \____ \
   /        \  ___/|  | |  |  /  |_> >
  /_______  /\___  >__| |____/|   __/
          \/     \/           |__|
*/
void setup() {

  //enable LDO
  pinMode(INH_EN_LDO, INPUT);

  //important DELAY FOR BOOTING
  //reduce charge on the LDO before activate ESP32
  delay(2000);

  Temp_0 = 25 + 273.15;

  //led
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  //blink 20 fois rapide au demarrage
  for (int kk = 0; kk < 20; kk++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(10);
    digitalWrite(LED_PIN, LOW);
    delay(40);
  }

  if (EMULATEUR == 0)
  {
    //Capteur NGL
    //Setup_IO(); //enable power
    Setup_PWM();
    Setup_ADC();
    Setup_I2C();
    Setup_SERIAL();

    //INIT SPI
    hspi = new SPIClass(HSPI);

    //clock miso mosi ss
    pinMode(HSPI_MISO, INPUT_PULLUP);  //HSPI SS pinMode(2, INPUT_PULLUP);

    //CS HIGH
    pinMode(HSPI_SS_AD5270_20K, OUTPUT);  //HSPI SS
    pinMode(HSPI_SS_AD5270_20K_IN, OUTPUT);  //HSPI SS
    digitalWrite(HSPI_SS_AD5270_20K, HIGH);
    digitalWrite(HSPI_SS_AD5270_20K_IN, HIGH);

    //alternatively route through GPIO pins
    hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS_AD5270_20K);  //SCLK, MISO, MOSI, SS
    //force pull up for POT 20K running
    pinMode(HSPI_MISO, INPUT_PULLUP);  //HSPI SS pinMode(2, INPUT_PULLUP);

    //mode 0 : CPOL=0 et CPHA=0.
    //mode 1 : CPOL=0 et CPHA=1.
    //mode 2 : CPOL=1 et CPHA=0.
    //mode 3 : CPOL=1 et CPHA=1.

    //SOFTWARE RESET POTENTIOMETER
    int_pot20k_input = 0;
    AD5270_WriteReg(SW_RST, 0x00);
    delay(10);
    floatRealValueROut = AD5270_WriteRDAC(val_pot_20k);
    delay(10);

    int_pot20k_input = 1;
    AD5270_WriteReg(SW_RST, 0x00);
    delay(10);
    floatRealValueRIn = AD5270_WriteRDAC(val_pot_10k);
    delay(10);

    int_pot20k_input = 0;
  }
  else
  {
    //EMULATEUR
    //INIT SPI
    hspi = new SPIClass(HSPI);

    //clock miso mosi ss
    pinMode(HSPI_MISO, INPUT_PULLUP);  //HSPI SS pinMode(2, INPUT_PULLUP);
    pinMode(HSPI_SS_AD5270_20K, OUTPUT);  //HSPI SS

    //alternatively route through GPIO pins
    hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS_AD5270_20K);  //SCLK, MISO, MOSI, SS
    //force pull up for POT 20K running
    pinMode(HSPI_MISO, INPUT_PULLUP);  //HSPI SS pinMode(2, INPUT_PULLUP);

    //mode 0 : CPOL=0 et CPHA=0.
    //mode 1 : CPOL=0 et CPHA=1.
    //mode 2 : CPOL=1 et CPHA=0.
    //mode 3 : CPOL=1 et CPHA=1.

    Serial.begin(1500000);
    Serial.println("The device (NGL Sensors) started, now you can pair it with bluetooth!");
  }

  //________________________BLE______________________
  // Create the BLE Device
  if (EMULATEUR == 0)
  {
    BLEDevice::init("NGL Sensors");
  }
  else
  {
    BLEDevice::init("Emulateur");
  }

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
/*
  .____
  |    |    ____   ____ ______
  |    |   /  _ \ /  _ \\____ \
  |    |__(  <_> |  <_> )  |_> >
  |_______ \____/ \____/|   __/
          \/            |__|
*/
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
      if (EMULATEUR == 0)
      {
        Setup_IO(); //enable power
      }
    }
  }


  if (EMULATEUR == 0)
  {
    // ********* CLOCK *********
    ledcWrite(pwmChannel_clock, 127); //1.65 V de temps haut --> clock

    // ********* READING ADC BATTERY VOLTAGE *********
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

    // ********* READING NTC TEMPERATURE VOLTAGE *********
    //Mesure ADC NTC  ADC ESP32
    VRT = analogRead(OUT_NTC_PIN);
    VRT = analogRead(OUT_NTC_PIN);
    VRT = analogRead(OUT_NTC_PIN);

    //MESURE TENSION NTC
    VR = VCC - VRT;
    //RT = VRT / (VR / R);               //Resistance of RT

    //Rntc=Rpont*Vin/Vout-Rpont
    //RT = (VRT-VR)*10000/VR;               //Resistance of RT
    RT = (10000 * VCC / VR) - 10000;

    ln = log(RT / RT0);
    TXX = (1 / ((ln / B) + (1 / Temp_0))); //Temperature from thermistor
    TXX = TXX - 273.15;                 //Conversion to Celsius

    //  Serial.print("Temp = ");
    Serial.print(VRT);  //ADC value
    Serial.print(",");
    Serial.print(TXX); //Temperature value
    Serial.print(",");

    // ********* I²C *********
    //***** Channel 1
    // ********* READING CONDUCTIVIMETER/VIBRATION ADC16bit *********
    // OUT_SENS output

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
    }



    //***** Channel 2
    // ********* READING AMPLIFIER DIFFERENTIAL ADC16bit *********
    //OUT_DIFF OUTPUT
    // Essai en int  16 Signed: int16_t
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
    }

    // ********* Conversion en Volt *********
    fOutSens_V = (VREF_ADC / 32767 ) * value; // Attention vref = 2.048 V
    //Serial.print("OUT_SENS = ");
    Serial.print(fOutSens_V);

    // ********** SEND VALUE potentiometre 20K **********************
    Serial.print(",");
    Serial.print(val_pot_20k);

    // ********** SEND VALUE potentiometre 10K **********************
    Serial.print(",");
    Serial.print(val_pot_10k);
  }
  else
  {
    delay(200);
    //SIGNAL EMULATION for android debugging

    value2 = value2 + 64;
    if (value2 >= 65536) value2 = 0;

    //12bits
    VRT = VRT + 4;
    if (VRT >= 4096) VRT = 0;

    Serial.print(value2);
    Serial.print(",");
    Serial.print(VRT);
    Serial.print(",");
    Serial.print(val_pot_20k);
    Serial.print(",");
    Serial.print(val_pot_10k);
  }

  //------------------ BLE -------------------------
  if (deviceConnected) {

    //blink led
    digitalWrite(LED_PIN, HIGH);
    delay(10);
    digitalWrite(LED_PIN, LOW);

    // Let's convert the value to a char array:
    //char txString[16]; // make sure this is big enuffz

    if (EMULATEUR == 0)
    {
      //dtostrf(value3_vib_cond, 7, 0, txString); // float_val, min_width, digits_after_decimal, char_buffer
    }
    else
    {
      //dtostrf(value2, 7, 0, txString); // float_val, min_width, digits_after_decimal, char_buffer
    }
    /*
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

      pCharacteristic->setValue(txString);
    */
    //    sprintf(txStringSend, "%.2f,%.2f,%.3f", averageVoltage, tdsValue, conductivity);
    sprintf(txStringSendBle, "%.0f,%d,%.1f,%d,%.0f,%.0f", value3_vib_cond, valueDiff, VRT, OutBatValue, floatRealValueRIn, floatRealValueROut);

    pCharacteristic->setValue(txStringSendBle);
    pCharacteristic->notify();

    delay(DELAY_LOOP_BLE_ACTIVE);

    Serial.print(",ble:");
    Serial.println(txStringSendBle);

    //---------- BLE RECEIVE ---------------------------
    if (boolReceive == true)
    {
      boolReceive = false;

      Serial.println("----------------------------------");
      Serial.print("Received BLE: ");
      Serial.println(inStringBleReceive);

      virgCharIndex = inStringBleReceive.indexOf(',');

      if (virgCharIndex != -1)
      {
        //format 10k potentiometer
        StringRes10k = inStringBleReceive.substring(0, virgCharIndex);
        Serial.print("IN-10k: ");
        Serial.println(StringRes10k);

        StringRes20k = inStringBleReceive.substring((virgCharIndex + 1), inStringBleReceive.length());
        Serial.print("OUT-20k: ");
        Serial.println(StringRes20k);

        val_pot_20k = StringRes20k.toInt();
        //controle 20k potentiometer
        if (val_pot_20k >= 20000) val_pot_20k = 20000;
        if (val_pot_20k <= 0)     val_pot_20k = 0;

        val_pot_10k = StringRes10k.toInt();
        //controle 10k potentiometer
        if (val_pot_10k >= 10000) val_pot_10k = 10000;
        if (val_pot_10k <= 0)     val_pot_10k = 0;

        inStringBleReceive = "";
        StringRes20k = "";
        StringRes10k = "";

        if (EMULATEUR == 0)
        {
          //ADD AD5270 potentiometer 20k externe limited to 10k
          //alternatively route through GPIO pins
          int_pot20k_input = 1;
          hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS_AD5270_20K_IN);  //SCLK, MISO, MOSI, SS
          pinMode(HSPI_MISO, INPUT_PULLUP);  //HSPI SS pinMode(2, INPUT_PULLUP);
          floatRealValueRIn = AD5270_WriteRDAC(val_pot_10k);

          //change AD5270 potentimeter 20k
          //alternatively route through GPIO pins
          int_pot20k_input = 0;
          hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS_AD5270_20K);  //SCLK, MISO, MOSI, SS
          pinMode(HSPI_MISO, INPUT_PULLUP);  //HSPI SS pinMode(2, INPUT_PULLUP);
          floatRealValueROut = AD5270_WriteRDAC(val_pot_20k);


          //change AD5270 potentimeter 20k
          //          hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS_MAX5481_10K);  //SCLK, MISO, MOSI, SS
          //          pinMode(HSPI_MISO, INPUT_PULLUP);  //HSPI SS pinMode(2, INPUT_PULLUP);
          //          val_pot_10k = WriteMax5481Pot(val_pot_10k, 0); // HSPI ok à l'ocillo
        }
        else
        {
          hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS_MAX5481_10K);  //SCLK, MISO, MOSI, SS
          pinMode(HSPI_MISO, INPUT_PULLUP);  //HSPI SS pinMode(2, INPUT_PULLUP);
          val_pot_10k = WriteMax5481Pot(val_pot_10k, 0); // to observe algo pot 10k on 5bit
        }
      }
      else
      {
        //no virgule bad format
        Serial.println("no change potentiometer");
      }

      Serial.println("----------------------------------");

    }
  }
  else
  {
    //device BLE not connected
    Serial.println();
  }


  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }


  //Connecting BLE DEVICE
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    Serial.println("Device connected ;-)");
  }

  //DISCONNECTED BLE DEVICE
  if (!deviceConnected)
  {
    //low reading cycle for reduce power
    delay(DELAY_LOOP_BLE_NOTACTIVE);
    //blink led
    digitalWrite(LED_PIN, HIGH);
    delay(5);
    digitalWrite(LED_PIN, LOW);
  }
}



/*
  _____________________________           _______________   __
  \______   \_____  \__    ___/           \_____  \   _  \ |  | __
   |     ___//   |   \|    |      ______   /  ____/  /_\  \|  |/ /
   |    |   /    |    \    |     /_____/  /       \  \_/   \    <
   |____|   \_______  /____|              \_______ \_____  /__|_ \
                    \/                            \/     \/     \/
*/
////POTENTIOMMETER functions AD5270

/**
   @brief Compute for the nearest RDAC value from given resistance
   @param resistance - resistor
   @return RDAC value - closest possible to given resistance
*/
uint16_t AD5270_CalcRDAC(float resistance) {
  return ((uint16_t)((resistance / MAX_RESISTANCE) * 1023.0));
}



/**
   @brief sets a new value for the RDAC
   @param resistance new value for the resistance
   @return actual value of the resistance in the RDAC
*/

float AD5270_WriteRDAC(float resistance)
{
  uint16_t setValue = 0x00;

  uint16_t RDAC_Val = AD5270_CalcRDAC(resistance);
  //RDAC_Val = 1023;
  Serial.print("R");
  Serial.print(resistance);
  Serial.print("-");
  Serial.print("DAC");
  Serial.print(RDAC_Val);
  Serial.print("-");
  //RDAC_Value = (float)((RDAC_val * MAX_RESISTANCE) / 1024.0);  // inverse operation to get actual resistance in the RDAC

  //setValue = AD5270_ReadReg(READ_CTRL_REG);
  //    RDAC_val = AD5270_CalcRDAC(resistance)
  //    spi.xfer2([WRITE_CTRL_REG, RDAC_WRITE_PROTECT])   WRITE_CTRL_REG      = 0x1C
  //    AD5270_WriteReg(WRITE_RDAC, RDAC_val);  WRITE_RDAC          = 0x04

  uint8_t ui8Adress = (WRITE_RDAC | ((uint8_t)((RDAC_Val >> 8) & 0x00FF)));
  sprintf(buffer_pot, "%02X", ui8Adress);
  Serial.print(buffer_pot);
  Serial.print("-");
  sprintf(buffer_pot, "%02X", (uint8_t)(RDAC_Val & 0x00FF));
  Serial.print(buffer_pot);

  // RDAC register write protect -  allow update of wiper position through digital interface
  AD5270_WriteReg(WRITE_CTRL_REG, 0x02);  //UNLOCK RDAC_WRITE_PROTECT
  delayMicroseconds(10);
  // write data to the RDAC register
  AD5270_WriteReg(ui8Adress , (uint8_t)(RDAC_Val & 0x00FF));
  delayMicroseconds(10);

  // RDAC register read RDAC
  RreadADC = AD5270_ReadRDAC();
  delayMicroseconds(10);

  floatvallocaltemp = ((float)RDAC_Val * (MAX_RESISTANCE / 1023.0));

  if (RreadADC != floatvallocaltemp)
  {
    Serial.println("!!! WARNING !!!! PROBLEM POT PROGRAMMING");
    Serial.print("RreadADC:");
    Serial.println(RreadADC);
    Serial.print("RDAC_Val:");
    Serial.println(floatvallocaltemp);
  }
  else
  {
    Serial.println("POT GOOD PROGRAMMING");
    Serial.print("RreadADC:");
    Serial.println(RreadADC);
    Serial.print("RDAC_Val:");
    Serial.println(floatvallocaltemp);
  }

  //MISO high impedance
  AD5270_WriteReg(0x80, 0x01);   //0xAA dummy
  delayMicroseconds(10);
  AD5270_WriteReg(0x00, 0x00);   //0xAA dummy
  delayMicroseconds(10);

  return (float)floatvallocaltemp;
}



/**
   Reads the RDAC register
   @return RDAC resistor value
*/
float AD5270_ReadRDAC(void) {
  uint16_t RDAC_val;
  uint16_t result = 0;
  uint16_t result2 = 0;

  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));

  if (int_pot20k_input == 0)
  {
    digitalWrite(HSPI_SS_AD5270_20K, LOW);
  }
  else
  {
    digitalWrite(HSPI_SS_AD5270_20K_IN, LOW);
  }

  result2 = hspi->transfer(READ_CTRL_REG);
  result = hspi->transfer(0xAA);

  if (int_pot20k_input == 0)
  {
    digitalWrite(HSPI_SS_AD5270_20K, HIGH);
  }
  else
  {
    digitalWrite(HSPI_SS_AD5270_20K_IN, HIGH);
  }
  hspi->endTransaction();


  RDAC_val = ((result2 & 0x03) << 8) | result;
  Serial.println();
  Serial.print("RDAC-READ:");
  sprintf(buffer_pot, "%04X", RDAC_val);
  Serial.println(buffer_pot);

  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  if (int_pot20k_input == 0)
  {
    digitalWrite(HSPI_SS_AD5270_20K, LOW);
  }
  else
  {
    digitalWrite(HSPI_SS_AD5270_20K_IN, LOW);
  }

  hspi->transfer(READ_CTRL_REG);
  result2 = hspi->transfer(0xAA);

  if (int_pot20k_input == 0)
  {
    digitalWrite(HSPI_SS_AD5270_20K, HIGH);
  }
  else
  {
    digitalWrite(HSPI_SS_AD5270_20K_IN, HIGH);
  }
  hspi->endTransaction();

  //RDAC_val = (result2 << 8) | result;

  //RDAC_val = AD5270_ReadReg(READ_CTRL_REG);
  //RDAC_val &= 0x03FF;

  return ((float)(RDAC_val) * (MAX_RESISTANCE / 1023.0));
}



/**
   @brief Puts the AD5270 SDO line in to Hi-Z mode
  @return none
*/
// void AD5270_Set_SDO_HiZ(void)
// {
//    uint8_t data1[2] = {HI_Zupper, HI_Zlower};
//    uint8_t data2[2] = {NO_OP, NO_OP};

//    digitalWrite(AD5270_CS_PIN, LOW);
//    SPI_Write(data1, 2, AD5270);
//    SPI_Write(data1, 2, AD5270);
//    digitalWrite(AD5270_CS_PIN, HIGH);

//    digitalWrite(AD5270_CS_PIN, LOW);
//    SPI_Write(data2, 2, AD5270);
//    SPI_Write(data2, 2, AD5270);
//    digitalWrite(AD5270_CS_PIN, HIGH);
// }



uint16_t AD5270_ReadReg(uint8_t command) {

  uint8_t data[2];
  uint16_t result = 0;
  uint16_t result2 = 0;

  data[0] = (command & 0x3C);

  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  digitalWrite(HSPI_SS_AD5270_20K, LOW);

  hspi->transfer(data[0]);
  result = hspi->transfer(0x00);

  digitalWrite(HSPI_SS_AD5270_20K, HIGH);
  hspi->endTransaction();

  delayMicroseconds(10);

  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  digitalWrite(HSPI_SS_AD5270_20K, LOW);

  hspi->transfer(data[0]);
  result2 = hspi->transfer(0x00);

  digitalWrite(HSPI_SS_AD5270_20K, HIGH);
  hspi->endTransaction();

  result = (result2 << 8) | result;

  return result;
}



/**
    Enables the 50TP memory programming
*/
void AD5270_Enable_50TP_Programming(void) {
  uint8_t regVal = (uint8_t)AD5270_ReadReg(READ_CTRL_REG);
  AD5270_WriteReg(WRITE_CTRL_REG, (regVal | PROGRAM_50TP_ENABLE));  // RDAC register write protect -  allow update of wiper position through digital interface
}



/**
    Stores current RDAC content to the 50TP memory
*/
void AD5270_Store_50TP(void) {
  AD5270_WriteReg(STORE_50TP, 0);
  delay(WRITE_OPERATION_50TP_TIMEOUT);
}



/**
   Disables the 50TP memory programming
*/
void AD5270_Disable_50TP_Programming(void) {
  uint8_t regVal = AD5270_ReadReg(READ_CTRL_REG);
  AD5270_WriteReg(WRITE_CTRL_REG, (regVal & (~PROGRAM_50TP_ENABLE)));
}



/**
   @brief Writes 16bit data to the AD5270 SPI interface
   @param data to be written
   @return data returned by the AD5270
*/
void AD5270_WriteReg(uint8_t command, uint16_t value) {
  uint8_t data[2];

  data[0] = (command & 0x3C);
  data[0] |= (uint8_t)((value & 0x0300) >> 8);

  data[1] = (uint8_t)(value & 0x00FF);

  //SPI_Write(data,2, AD5270);

  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  if (int_pot20k_input == 0)
  {
    digitalWrite(HSPI_SS_AD5270_20K, LOW);
  }
  else
  {
    digitalWrite(HSPI_SS_AD5270_20K_IN, LOW);
  }

  //hspi->transfer((byte)((stuff & 0x00FF0000)>>16));
  hspi->transfer((byte)command);
  hspi->transfer((byte)data[1]);

  if (int_pot20k_input == 0)
  {
    digitalWrite(HSPI_SS_AD5270_20K, HIGH);
  }
  else
  {
    digitalWrite(HSPI_SS_AD5270_20K_IN, HIGH);
  }
  hspi->endTransaction();
}



/**
   Reads the last programmed value of the 50TP memory
   @return last programmed value
*/
uint8_t AD5270_Read_50TP_LastAddress(void) {
  uint8_t ret_val;

  AD5270_WriteReg(READ_50TP_ADDRESS, 0);
  ret_val = AD5270_ReadReg(NO_OP);

  return ret_val;
}



/**
   Reads the content of a 50TP memory address
   @param address memory to be read
   @return value stored in the 50TP address
*/
uint16_t AD5270_Read_50TP_memory(uint8_t address) {
  uint16_t ret_val;

  AD5270_WriteReg(READ_50TP_CONTENTS, address);
  ret_val = AD5270_ReadReg(NO_OP);

  return ret_val;
}



/**
   Resets the wiper register value to the data last written in the 50TP
*/
void AD5270_ResetRDAC(void) {
  AD5270_WriteReg(SW_RST, 0);
}



/**
   Changes the device mode, enabled or shutdown
   @param mode - new mode of the device
*/
void AD5270_ChangeMode(AD5270Modes_t mode) {

  AD5270_WriteReg(SW_SHUTDOWN, (uint16_t)(mode));
}



/*
  _____________________________            ___________   __
  \______   \_____  \__    ___/           /_   \   _  \ |  | __
   |     ___//   |   \|    |      ______   |   /  /_\  \|  |/ /
   |    |   /    |    \    |     /_____/   |   \  \_/   \    <
   |____|   \_______  /____|               |___|\_____  /__|_ \
                    \/                                \/     \/
*/
//fonctions potentiometer MAX5481

int WriteMax5481Pot(long int ResPotValue, int int_save_eeprom)
{
  int matStepPotMax5481[32] = {0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448, 480, 512, 544, 576, 608, 640, 672, 704, 736, 768, 800, 832, 864, 896, 928, 960, 992};
  int matValPotMax5481[32] = {939, 1253, 1564, 1883, 2200, 2510, 2830, 3140, 3460, 3770, 4080, 4400, 4720, 5030, 5340, 5660, 5970, 6290, 6600, 6920, 7230, 7550, 7850, 8170, 8490, 8810, 9110, 9430, 9750, 10060, 10360, 10690};

  long int cloneStuff = ResPotValue;
  int minEcart = 65000;
  int Ecart = 0;
  int indicePot = 0;

  Serial.println("--------- CONFIG POT 10K MAX5481 -------");
  Serial.print("STUFF:");
  Serial.println(ResPotValue);

  indicePot = 0;
  for (int jj = 0; jj < 32; jj++)
  {
    Ecart = matValPotMax5481[jj] - ResPotValue;
    if (Ecart < 0) Ecart = -Ecart;

    if (Ecart < minEcart)
    {
      minEcart = Ecart;
      indicePot = jj;
    }

    Serial.print("ind:");
    Serial.print(indicePot);
    Serial.print("-Ecart:");
    Serial.print(Ecart);
    Serial.print("-EcartMin:");
    Serial.print(minEcart);
    Serial.print("-MAT_POT:");
    Serial.println(matValPotMax5481[jj]);
  }

  ResPotValue = matStepPotMax5481[indicePot];

  Serial.print("STUFF final:");
  Serial.println(ResPotValue);

  if (EMULATEUR == 0)
  {
    //Send SPI frame
    ResPotValue = ResPotValue << 6;

    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_SS_MAX5481_10K, LOW);

    hspi->transfer(0x00);
    hspi->transfer((byte)((ResPotValue & 0x0000FF00) >> 8));
    hspi->transfer((byte)(ResPotValue & 0x000000FF));  //ENVOIE Message : au lieu de transferer on va DPOT !

    digitalWrite(HSPI_SS_MAX5481_10K, HIGH);
    hspi->endTransaction();

    delay(10);

    if (int_save_eeprom == 1)
    {
      hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
      digitalWrite(HSPI_SS_MAX5481_10K, LOW);

      hspi->transfer(0x20);  //ENVOIE Message : au lieu de transferer on va DPOT !

      digitalWrite(HSPI_SS_MAX5481_10K, HIGH);
      hspi->endTransaction();

      delay(13);
    }
  }//if not emulator

  else
  {
    //emulateur
    //Send SPI frame
    ResPotValue = ResPotValue << 6;

    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_SS_MAX5481_10K, LOW);

    hspi->transfer(0x00);
    hspi->transfer((byte)((ResPotValue & 0x0000FF00) >> 8));
    hspi->transfer((byte)(ResPotValue & 0x000000FF));  //ENVOIE Message : au lieu de transferer on va DPOT !

    digitalWrite(HSPI_SS_MAX5481_10K, HIGH);
    hspi->endTransaction();

    delay(10);

    if (int_save_eeprom == 1)
    {
      hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
      digitalWrite(HSPI_SS_MAX5481_10K, LOW);

      hspi->transfer(0x20);  //ENVOIE Message : au lieu de transferer on va DPOT !

      digitalWrite(HSPI_SS_MAX5481_10K, HIGH);
      hspi->endTransaction();

      delay(13);
    }
  }



  return matValPotMax5481[indicePot];
}
