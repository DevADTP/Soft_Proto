
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

#include <SPI.h>
#include "AD5270.h"


/*==============================================================================
**                             Local Defines                                    **
  ================================================================================*/
#define EMULATEUR 1

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


//SPI POTENTIOMETER
//20K AD5270 (spi IN/OUT)
// carte NGL
#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_SCLK 14
#define HSPI_SS 16

static const int spiClk = 500000;  // 1 MHz
int val_pot_20k = 0;
int val_pot_10k = 0;
char buffer_pot[30];
int virgCharIndex = 0;

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
    Setup_SERIAL();

    //INIT SPI
    hspi = new SPIClass(HSPI);

    //clock miso mosi ss
    pinMode(HSPI_MISO, INPUT_PULLUP);  //HSPI SS pinMode(2, INPUT_PULLUP);
    pinMode(HSPI_SS, OUTPUT);  //HSPI SS

    //alternatively route through GPIO pins
    hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);  //SCLK, MISO, MOSI, SS

    pinMode(HSPI_MISO, INPUT_PULLUP);  //HSPI SS pinMode(2, INPUT_PULLUP);

    //mode 0 : CPOL=0 et CPHA=0.
    //mode 1 : CPOL=0 et CPHA=1.
    //mode 2 : CPOL=1 et CPHA=0.
    //mode 3 : CPOL=1 et CPHA=1.

    AD5270_WriteReg(SW_RST, 0x00);  //SOFTWARE RESET POTENTIOMETER
    delay(10);

    AD5270_WriteRDAC(val_pot_20k);
  }
  else
  {
    Serial.begin(1500000);
    Serial.println("The device (NGL Sensors) started, now you can pair it with bluetooth!");
  }

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
    Serial.print(fOutSens_V);

    // ********** potentiometre 20K ***************************
    Serial.print(",");
    Serial.println(val_pot_20k);
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

    pCharacteristic->setValue(txString);

    pCharacteristic->notify();
    delay(500);
    
    Serial.print(",");
    Serial.println(txString);

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
        //format 20k potentiometer
        StringRes20k = inStringBleReceive.substring(0, virgCharIndex);
        Serial.print("20k: ");
        Serial.println(StringRes20k);

        StringRes10k = inStringBleReceive.substring((virgCharIndex+1), inStringBleReceive.length());
        Serial.print("10k: ");
        Serial.println(StringRes10k);

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
          //change potentimeter 20k
          AD5270_WriteRDAC(val_pot_20k);
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
      Serial.println();
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
float RreadADC = 0;
uint16_t AD5270_WriteRDAC(float resistance) {

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

  Serial.print("ReadADC ");
  Serial.println((int)RreadADC);

  //MISO high impedance
  AD5270_WriteReg(0x80, 0x01);   //0xAA dummy
  delayMicroseconds(10);
  AD5270_WriteReg(0x00, 0x00);   //0xAA dummy
  delayMicroseconds(10);

  return RDAC_Val;
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
  digitalWrite(HSPI_SS, LOW);

  hspi->transfer(READ_CTRL_REG);
  result = hspi->transfer(0xAA);

  digitalWrite(HSPI_SS, HIGH);
  hspi->endTransaction();

  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  digitalWrite(HSPI_SS, LOW);

  hspi->transfer(READ_CTRL_REG);
  result2 = hspi->transfer(0xAA);

  digitalWrite(HSPI_SS, HIGH);
  hspi->endTransaction();

  RDAC_val = (result2 << 8) | result;

  //RDAC_val = AD5270_ReadReg(READ_CTRL_REG);
  //RDAC_val &= 0x03FF;

  return (((float)(RDAC_val) * MAX_RESISTANCE) / 1024.0);
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
  digitalWrite(HSPI_SS, LOW);

  hspi->transfer(data[0]);
  result = hspi->transfer(0x00);

  digitalWrite(HSPI_SS, HIGH);
  hspi->endTransaction();

  delayMicroseconds(10);

  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  digitalWrite(HSPI_SS, LOW);

  hspi->transfer(data[0]);
  result2 = hspi->transfer(0x00);

  digitalWrite(HSPI_SS, HIGH);
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
  digitalWrite(HSPI_SS, LOW);


  //hspi->transfer((byte)((stuff & 0x00FF0000)>>16));
  hspi->transfer((byte)command);
  hspi->transfer((byte)data[1]);

  digitalWrite(HSPI_SS, HIGH);
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
