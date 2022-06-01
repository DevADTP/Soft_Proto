


/*============================== (c) 2022 ADTP ================================
** File Name   :  NGL_FireBeetle_V1                                          **
** Author      :  Benoit                                                     **
** Created on  :  Jui 1, 2022                                                **
**---------------------------------------------------------------------------**
** Description : Controls the NGL_Proto PCB                                  **
*                 with ESP32 WROOM-32D-N16                                   **
=============================================================================*/



/*=============================================================================
**                                 PINOUT                                     **
=============================================================================*/

/*
LED BLUE  --> IO25
CLOCK_GEN --> IO19

MISO      --> IO12
MOSI      --> IO13
CLK       --> IO14
CS        --> IO15

SCL       --> IO21
SDA       --> IO22

NEO       --> IO26

OUT_NTC   --> SENSOR_VP --> IO36 
OUT_BAT   --> SENSOR_VN --> IO39

Flash par UART ! 
ESP32 uniquement en 3.3 V ! 


Hardware:

Pont div sur VBAT: VS/VIN = 0.77
Full charge à 4.2 V

ADC MCP3426A0-E_SN
16 Bits _ 15 SPS

INFOS:
Librairie ESP32 Utilisé avec comme board ESP32_FireBeetle pour l'ESP Wroom
https://techtutorialsx.com/2017/06/05/esp-wroom-32-uploading-a-program-with-arduino-ide/

*/

#include <Wire.h>
#include <MCP342x.h>


  /*==============================================================================
**                             Local Defines                                    **
================================================================================*/
#define LED   25  
#define CLOCK 19

#define DELAY_PWM 10

#define BRIGHTNESS_LED 69    // 8 Bits

#define I2C_SDA 21
#define I2C_SCL 22

 /*===============================================================================
 **                            Global Variables                                 **
 ===============================================================================*/
//int ledPin = 2;

int pwmChannel = 0; //Choisit le canal 0
int pwmChannel_clock = 1; //Choisit le canal 0


int frequence = 1000; //Fréquence PWM de 1 KHz
int resolution = 8; // Résolution de 8 bits, 256 valeurs possibles

int pwmPin = 25;
int Pin_clock = 19;

// Le OUT BATTERIE est connecté au GPIO 36 (Pin VP)
const int Out_Bat_PIN = 39;
const int Out_Ntc_PIN = 36;

int OutBatValue = 0;
int OutNtcValue = 0;

float OutBatVolt = 0;

void setup() {

  /*
// Set LED as output
    pinMode(LED, OUTPUT);

      */
  Serial.begin(115200);

   //----------- PWN LED ----------- 
      // Configuration du canal 0 avec la fréquence et la résolution choisie
    ledcSetup(pwmChannel, frequence, resolution);

    // Assigne le canal PWM au pin 25
    ledcAttachPin(pwmPin, pwmChannel);

    // Créer la tension en sortie choisi
    ledcWrite(pwmChannel, 127); //1.65 V

    //----------- PWN Clock ----------- 

      // Configuration du canal 0 avec la fréquence et la résolution choisie
    ledcSetup(pwmChannel_clock, frequence, resolution);

    // Assigne le canal PWM au 
    ledcAttachPin(Pin_clock, pwmChannel_clock);

    // Créer la tension en sortie choisi
    ledcWrite(pwmChannel_clock, BRIGHTNESS_LED); //1.65 V


    //----------- ADC ----------- 
    pinMode(Out_Bat_PIN,INPUT_PULLUP);
    pinMode(Out_Ntc_PIN,INPUT_PULLUP);
    
    //----------- I²C ----------- 
    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.println("\nI2C Scanner");
}
 
void loop() {

  /*
    Serial.print("Hello");
    digitalWrite(LED, HIGH);
    
    delay(100);
    
    Serial.println(" world!");
    digitalWrite(LED, LOW);
    
    delay(100);

     */


         // ********* DIMMING LED *********
    for(int dutyCycle = 0; dutyCycle <= 169; dutyCycle++){
      
        ledcWrite(pwmChannel, dutyCycle); //1.65 V
        delay(DELAY_PWM);
                                                         }

             ledcWrite(pwmChannel_clock, 127); //1.65 V de temps haut --> clock  


       // ********* READING ADC *********
          // Tension batterie
        OutBatValue = analogRead(Out_Bat_PIN);
        OutBatVolt  = OutBatValue*(3.3/4096)/0.77; //Conversion en volt des 12 Bits
        Serial.print("Out_Bat = ");
        Serial.print(OutBatVolt);
        Serial.println(" V ");
        delay(250);

          // Tension NTC
        OutNtcValue = analogRead(Out_Ntc_PIN);
        Serial.print("Out_Ntc =");
        Serial.println(OutNtcValue);
        delay(250);
       // ********* I²C *********      

         byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(5000);  

}
