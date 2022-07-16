/*****
 * max5481.ino
 *
 * Created on: 11-16-2016
 *     Author: Robert F. Chapman <Robert.F.Chapman@gmail.com>
 *             Copyright (c) 2016-2019 Robert F. Chapman
 *  
 *  This file is part of MAX5481 Library for Arduino Micro(s).
 *
 *  This Library is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.

 *  This Library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Foobar.  If not, see <https://www.gnu.org/licenses/>.

M5STACK  ATOM LITE connexion
                ------------
  GYRO -- 3V3  | 3V3        | 
  GYRO <- CS   | G22    G21 | 
  GYRO <- MOSI | G19    G25 | 
  GYRO <- CLK  | G23    5V  | 
  GYRO -> MISO | G33    GND | -- GYRO
                ------------
 */
#include <SPI.h>

#include <MAX5481.h>
#include "AD5270.h"

// carte NGL
//#define HSPI_MISO 12
//#define HSPI_MOSI 13
//#define HSPI_SCLK 14
//#define HSPI_SS 17

//Atom lite
#define HSPI_MISO 33
#define HSPI_MOSI 19
#define HSPI_SCLK 23
#define HSPI_SS 22


#define DELAY_LOOP 1000
static const int spiClk = 1000000;  // 1 MHz 1000000; // 1 MHz

SPIClass* hspi = NULL;
SPIClass* vspi = NULL;

long int val_pot = 0;
char buffer_pot[30];

int val_uart = 0;

//#define DPIN 5  // Digital Pin 5 on Arduino


MAX5481 DPOT(HSPI_MOSI);


void hspiCommand(long int stuff, int int_save_eeprom);

void setup() {
  Serial.begin(115200);
  //initialise two instances of the SPIClass attached to VSPI and HSPI respectively

  hspi = new SPIClass(HSPI);

  //clock miso mosi ss

  //alternatively route through GPIO pins
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);  //SCLK, MISO, MOSI, SS

  //set up slave select pins as outputs as the Arduino API
  //doesn't handle automatically pulling SS low

  pinMode(HSPI_SS, OUTPUT);  //HSPI SS

  // This bit of code can be removed and is here only for
  // Demonstration purpouses.

  hspiCommand(val_pot, 1);  // HSPI ok à l'ocillo

  // hspiCommandSetup();
}


int i = 0;
void loop() {

  delay(1000);

  AD5270_WriteRDAC(val_pot);

    //while (Serial.available() > 0) {

    // look for the next valid integer in the incoming serial stream:
    //val_uart = Serial.parseInt();
    //Serial.print("uart:");
    //Serial.println(val_uart);
    //val_pot=val_uart;

    if (val_pot >= 1024) val_pot = 1023;

  // look for the newline. That's the end of your sentence:
  //if (Serial.read() == '\n') {

  Serial.print(val_pot);
  Serial.print("-");

  sprintf(buffer_pot, "%06X", (val_pot << 6));
  Serial.print(buffer_pot);
  Serial.print("-");

  sprintf(buffer_pot, "%02X", (byte)(((val_pot << 6) & 0x00FF0000) >> 16));
  Serial.print(buffer_pot);
  Serial.print("-");

  sprintf(buffer_pot, "%02X", (byte)(((val_pot << 6) & 0x0000FF00) >> 8));
  Serial.print(buffer_pot);
  Serial.print("-");

  sprintf(buffer_pot, "%02X", (byte)((val_pot << 6) & 0x000000FF));
  Serial.println(buffer_pot);


  //hspiCommand(val_pot,0);  // HSPI ok à l'ocillo

  // delay(DELAY_LOOP);
  delay(1000);
  //DPOT.setWiper(250);   // Sets the wiper to 250
  //delay(2500);

  val_pot = val_pot + 1;  //32 pour etre linéaire et etre sur le bord de la vague

  if (val_pot >= 1024) val_pot = 0;

  //}  //if receive uart

  // }  //while
}

void hspiCommand(long int stuff, int int_save_eeprom) {
  //byte stuff = 0b10101010;
  stuff = stuff << 6;

  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(HSPI_SS, LOW);


  //hspi->transfer((byte)((stuff & 0x00FF0000)>>16));
  hspi->transfer(0x00);
  hspi->transfer((byte)((stuff & 0x0000FF00) >> 8));
  hspi->transfer((byte)(stuff & 0x000000FF));  //ENVOIE Message : au lieu de transferer on va DPOT !
  //DPOT.setWiper(val_pot);  // Sets the wiper to 1000
  //DPOT.writeWiper();    // Writes the value of the wiper to NVM

  digitalWrite(HSPI_SS, HIGH);
  hspi->endTransaction();

  delay(10);

  if (int_save_eeprom == 1) {
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_SS, LOW);

    hspi->transfer(0x20);  //ENVOIE Message : au lieu de transferer on va DPOT !

    digitalWrite(HSPI_SS, HIGH);
    hspi->endTransaction();

    delay(13);
  }
}


void hspiCommandSetup() {
  byte stuff = 0b10101010;

  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(HSPI_SS, LOW);

  DPOT.begin();      // Initializes the MAX5481
  DPOT.readWiper();  // Sets the wiper to the value in NVM
                     // This command should be run in setup to initialize the DPOT
                     // to it's last saved value.

  digitalWrite(HSPI_SS, HIGH);
  hspi->endTransaction();
}









/**
 * @brief Compute for the nearest RDAC value from given resistance
 * @param resistance - resistor
 * @return RDAC value - closest possible to given resistance
 */
uint16_t AD5270_CalcRDAC(float resistance) {
  return ((uint16_t)((resistance / MAX_RESISTANCE) * 1024.0));
}
/**
 * @brief sets a new value for the RDAC
 * @param resistance new value for the resistance
 * @return actual value of the resistance in the RDAC
 */
float AD5270_WriteRDAC(float resistance) {
  float RDAC_Value;

  uint16_t setValue;

  uint16_t RDAC_val = AD5270_CalcRDAC(resistance);

  RDAC_Value = (float)((RDAC_val * MAX_RESISTANCE) / 1024.0);  // inverse operation to get actual resistance in the RDAC

  setValue = AD5270_ReadReg(READ_CTRL_REG);

  AD5270_WriteReg(WRITE_CTRL_REG, (setValue | RDAC_WRITE_PROTECT));  // RDAC register write protect -  allow update of wiper position through digital interface
  AD5270_WriteReg(WRITE_RDAC, RDAC_val);                             // write data to the RDAC register
  AD5270_WriteReg(WRITE_CTRL_REG, setValue);                         // RDAC register write protect -  allow update of wiper position through digital interface

  return RDAC_Value;
}

/**
 * Reads the RDAC register
 * @return RDAC resistor value
 */
float AD5270_ReadRDAC(void) {
  uint16_t RDAC_val;

  RDAC_val = AD5270_ReadReg(READ_CTRL_REG);
  RDAC_val &= 0x03FF;

  return (((float)(RDAC_val)*MAX_RESISTANCE) / 1024.0);
}

/**
 *	@brief Puts the AD5270 SDO line in to Hi-Z mode
 *	@return none
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

  data[0] = (command & 0x3C);

  //SPI_Read(data, 2, AD5270);

  result = data[0];
  result = (result << 8) | data[1];

  return result;
}
/**
 *  Enables the 50TP memory programming
 */
void AD5270_Enable_50TP_Programming(void) {
  uint8_t regVal = (uint8_t)AD5270_ReadReg(READ_CTRL_REG);
  AD5270_WriteReg(WRITE_CTRL_REG, (regVal | PROGRAM_50TP_ENABLE));  // RDAC register write protect -  allow update of wiper position through digital interface
}

/**
 *  Stores current RDAC content to the 50TP memory
 */
void AD5270_Store_50TP(void) {
  AD5270_WriteReg(STORE_50TP, 0);
  delay(WRITE_OPERATION_50TP_TIMEOUT);
}

/**
 * Disables the 50TP memory programming
 */
void AD5270_Disable_50TP_Programming(void) {
  uint8_t regVal = AD5270_ReadReg(READ_CTRL_REG);
  AD5270_WriteReg(WRITE_CTRL_REG, (regVal & (~PROGRAM_50TP_ENABLE)));
}

/**
 * @brief Writes 16bit data to the AD5270 SPI interface
 * @param data to be written
 * @return data returned by the AD5270
 */
void AD5270_WriteReg(uint8_t command, uint16_t value) {
  uint8_t data[2];

  data[0] = (command & 0x3C);
  data[0] |= (uint8_t)((value & 0x0300) >> 8);

  data[1] = (uint8_t)(value & 0x00FF);

  //SPI_Write(data,2, AD5270);

  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(HSPI_SS, LOW);


  //hspi->transfer((byte)((stuff & 0x00FF0000)>>16));
  hspi->transfer((byte)data[0]);
  hspi->transfer((byte)data[1]);

  digitalWrite(HSPI_SS, HIGH);
  hspi->endTransaction();
}

/**
 * Reads the last programmed value of the 50TP memory
 * @return last programmed value
 */
uint8_t AD5270_Read_50TP_LastAddress(void) {
  uint8_t ret_val;

  AD5270_WriteReg(READ_50TP_ADDRESS, 0);
  ret_val = AD5270_ReadReg(NO_OP);

  return ret_val;
}

/**
 * Reads the content of a 50TP memory address
 * @param address memory to be read
 * @return value stored in the 50TP address
 */
uint16_t AD5270_Read_50TP_memory(uint8_t address) {
  uint16_t ret_val;

  AD5270_WriteReg(READ_50TP_CONTENTS, address);
  ret_val = AD5270_ReadReg(NO_OP);

  return ret_val;
}

/**
 * Resets the wiper register value to the data last written in the 50TP
 */
void AD5270_ResetRDAC(void) {
  AD5270_WriteReg(SW_RST, 0);
}

/**
 * Changes the device mode, enabled or shutdown
 * @param mode - new mode of the device
 */
void AD5270_ChangeMode(AD5270Modes_t mode) {

  AD5270_WriteReg(SW_SHUTDOWN, (uint16_t)(mode));
}