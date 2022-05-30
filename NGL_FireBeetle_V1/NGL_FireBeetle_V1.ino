

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
#define LED   2
#define CLOCK 19

 /*===============================================================================
 **                            Global Variables                                 **
 ===============================================================================*/
//int ledPin = 2;


void setup() {
// Set LED as output
    pinMode(LED, OUTPUT);

    
  Serial.begin(115200);
}
 
void loop() {
    Serial.print("Hello");
    digitalWrite(LED, HIGH);
    
    delay(500);
    
    Serial.println(" world!");
    digitalWrite(LED, LOW);
    
    delay(500);

}
