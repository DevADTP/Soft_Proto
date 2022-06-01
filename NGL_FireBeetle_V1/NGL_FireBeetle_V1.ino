

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

OUT_NTC   --> SENSOR_VP
OUT_BAT   --> SENSOR_VN

Flash par UART ! 
ESP32 uniquement en 3.3 V ! 


Hardware:

Pont div sur VBAT: VS/VIN = 0.77
Full charge à 4.2 V

INFOS:
Librairie ESP32 Utilisé avec comme board ESP32_FireBeetle pour l'ESP Wroom
https://techtutorialsx.com/2017/06/05/esp-wroom-32-uploading-a-program-with-arduino-ide/

*/

  /*==============================================================================
**                             Local Defines                                    **
================================================================================*/
#define LED   25  
#define CLOCK 19

#define DELAY_PWM 10

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

void setup() {

  /*
// Set LED as output
    pinMode(LED, OUTPUT);

    
  Serial.begin(115200);
  */

      // Configuration du canal 0 avec la fréquence et la résolution choisie
    ledcSetup(pwmChannel, frequence, resolution);

    // Assigne le canal PWM au pin 25
    ledcAttachPin(pwmPin, pwmChannel);

    // Créer la tension en sortie choisi
    ledcWrite(pwmChannel, 127); //1.65 V

    //-----------

      // Configuration du canal 0 avec la fréquence et la résolution choisie
    ledcSetup(pwmChannel_clock, frequence, resolution);

    // Assigne le canal PWM au 
    ledcAttachPin(Pin_clock, pwmChannel_clock);

    // Créer la tension en sortie choisi
    ledcWrite(pwmChannel_clock, 127); //1.65 V

    
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


         // Augmente la luminosité de la led
    for(int dutyCycle = 0; dutyCycle <= 169; dutyCycle++){
      
        ledcWrite(pwmChannel, dutyCycle); //1.65 V
        delay(DELAY_PWM);
                                                         }

             ledcWrite(pwmChannel_clock, 127); //1.65 V de temps haut --> clock  

}
