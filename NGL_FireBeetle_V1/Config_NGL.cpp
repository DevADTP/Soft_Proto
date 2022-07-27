#include "Config_NGL.h"
#include "Arduino.h"
#include "BluetoothSerial.h"

#include <MCP342x.h>
#include <Wire.h>

#define LED_PIN      25
#define CLOCK_PIN    32
#define MOTEUR_PIN   18
#define I2C_SDA_PIN  22
#define I2C_SCL_PIN  21
#define OUT_BAT_PIN  39
#define OUT_NTC_PIN  36
#define INH_EN_LDO   23

#define DELAY_PWM    10

#define CLOCK_FREQ   1500

#define BRIGHTNESS_LED 69    // 8 Bits

#define VREF_ADC 2.048
//#define Addr 0x68

#define RT0 30000   // Ω
#define B 3977      // K
#define VCC 3.30    //Supply voltage
#define R 10000  //R=10KΩ

// #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
// #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
// #endif

extern int pwmChannel; //Choisit le canal 0
extern int pwmChannel_clock; //Choisit le canal 0
extern int resolution; // Résolution de 8 bits, 256 valeurs possibles

// extern BluetoothSerial SerialBT;


void Setup_PWM(void)
{
  //----------- PWN LED -----------
  //      // Configuration du canal 0 avec la fréquence et la résolution choisie
  //    ledcSetup(pwmChannel, CLOCK_FREQ, resolution);
  //
  //    // Assigne le canal PWM au pin 25
  //    ledcAttachPin(LED_PIN, pwmChannel);

  //    // Créer la tension en sortie choisi
  //    ledcWrite(pwmChannel_clock, BRIGHTNESS_LED); //1.65 V
  //
  //    // Créer la tension en sortie choisi


  //----------- PWN Clock -----------

  // Configuration du canal 0 avec la fréquence et la résolution choisie
  ledcSetup(pwmChannel_clock, CLOCK_FREQ, resolution);

  // Assigne le canal PWM au
  ledcAttachPin(CLOCK_PIN, pwmChannel_clock);

  ledcWrite(pwmChannel, 127); //1.65 V
}

void Setup_IO(void)
{
  //enable LDO
  pinMode(INH_EN_LDO, OUTPUT);

  //power ON
  digitalWrite(INH_EN_LDO, HIGH);

}

void Setup_ADC(void)
{

  //----------- ADC -----------
  pinMode(OUT_BAT_PIN, INPUT); //3.8 V pour l'instant sur out BAT donc ne pas déclarer
  pinMode(OUT_NTC_PIN, INPUT);
  //pinMode(MOTEUR , OUTPUT);    // sets the digital pin 13 as output
}

void Setup_I2C(void)
{
  //----------- I²C -----------
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Reset devices
  MCP342x::generalCallReset();
  delay(1); // MC342x needs 300us to settle, wait 1ms
}

void Setup_SERIAL(void)
{
  Serial.begin(115200);

  Serial.println("The device (NGL Sensors) started, now you can pair it with bluetooth!");
}
