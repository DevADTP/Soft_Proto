


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

BOARD: ESP32 Arduino --> FireBeetle-ESP32
*/

#include <Wire.h>
#include <MCP342x.h>
//#include <WiFi.h>



  /*==============================================================================
**                             Local Defines                                    **
================================================================================*/
#define LED   25  
#define CLOCK 32
#define MOTEUR 18


#define DELAY_PWM 10
#define DELAY_LOOP 500

#define BRIGHTNESS_LED 69    // 8 Bits

#define I2C_SDA 22
#define I2C_SCL 21
//#define Addr 0x68

#define RT0 30000   // Ω
#define B 3977      // K
#define VCC 3.30    //Supply voltage
#define R 10000  //R=10KΩ
 /*===============================================================================
 **                            Global Variables                                 **
 ===============================================================================*/
//int ledPin = 2;

int pwmChannel = 0; //Choisit le canal 0
int pwmChannel_clock = 1; //Choisit le canal 0


int frequence = 1500; //Fréquence PWM de 1 KHz
int resolution = 8; // Résolution de 8 bits, 256 valeurs possibles

int pwmPin = 25;
int Pin_clock = 32;

// Le OUT BATTERIE est connecté au GPIO 36 (Pin VP)
const int Out_Bat_PIN = 39;
const int Out_Ntc_PIN = 36;

int OutBatValue = 0;
int OutNtcValue = 0;

float OutBatVolt = 0;
float fOutSens_V = 0;
float fOutDiff_V = 0;
float RT, VR, ln, TXX, Temp_0, VRT;

uint8_t address = 0x68;
MCP342x adc = MCP342x(address);



void setup() {
/*
  //********* Hello woorld de test ********
// Set LED as output
    pinMode(LED, OUTPUT);

     */
  Temp_0 = 25 + 273.15;  
      
  Serial.begin(115200);

   //----------- PWN LED ----------- 
      // Configuration du canal 0 avec la fréquence et la résolution choisie
    ledcSetup(pwmChannel, frequence, resolution);

    // Assigne le canal PWM au pin 25
    ledcAttachPin(pwmPin, pwmChannel);
    ledcAttachPin(MOTEUR, pwmChannel);

    // Créer la tension en sortie choisi
    ledcWrite(pwmChannel, 127); //1.65 V

    //----------- PWN Clock ----------- 

      // Configuration du canal 0 avec la fréquence et la résolution choisie
    ledcSetup(pwmChannel_clock, frequence, resolution);

    // Assigne le canal PWM au 
    ledcAttachPin(Pin_clock, pwmChannel_clock);

    // Créer la tension en sortie choisi
    ledcWrite(pwmChannel_clock, BRIGHTNESS_LED); //1.65 V

    //pinMode(MOTEUR , OUTPUT);    // sets the digital pin 13 as output


    //----------- ADC ----------- 
   // ******** pinMode(Out_Bat_PIN,INPUT_PULLUP); ******** //3.8 V pour l'instant sur out BAT donc ne pas déclarer
    pinMode(Out_Ntc_PIN,INPUT_PULLUP);
    
    //----------- I²C ----------- 
    pinMode(I2C_SCL, INPUT_PULLUP); 
    pinMode(I2C_SDA, INPUT_PULLUP);
    Wire.begin(I2C_SDA, I2C_SCL);

   // Reset devices
  MCP342x::generalCallReset();
  delay(1); // MC342x needs 300us to settle, wait 1ms
  /*
  // Check device present
  Wire.requestFrom(address, (uint8_t)1);
  if (!Wire.available()) {
    Serial.print("No device found at address ");
    Serial.println(address, HEX);
    while (1)
      ;
  }


 */



}
 
void loop() {

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
        VRT = analogRead(Out_Ntc_PIN);
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

       
        delay(250);
        
       // ********* I²C *********  

//***** Channel 1
  long value = 0;
  MCP342x::Config status;
  // Initiate a conversion; convertAndRead() will wait until it can be read
  uint8_t err = adc.convertAndRead(MCP342x::channel1, MCP342x::oneShot,
           MCP342x::resolution16, MCP342x::gain1,
           1000000, value, status);
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
//  MCP342x::Config status;
  // Initiate a conversion; convertAndRead() will wait until it can be read
  uint8_t err2 = adc.convertAndRead(MCP342x::channel2, MCP342x::oneShot,
           MCP342x::resolution16, MCP342x::gain1,
           1000000, valueDiff, status);
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


    fOutSens_V = (3.3/32767 )* value;
    Serial.print("OUT_SENS = ");
    Serial.print(fOutSens_V);
    Serial.print(" V ");
    Serial.print(" \t ");

    fOutDiff_V = (3.3/32767 )* valueDiff;
    Serial.print("OUT_DIFF = ");
    Serial.print(fOutDiff_V);
    Serial.println(" V "); //dernier ln
   
    delay(DELAY_LOOP) ;
    // Voir pour plus de chiffre sur le float
    
   
}       
         
