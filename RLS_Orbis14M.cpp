/*!
 * @file RLS_Orbis14M.cpp
 *
 * @section intro_sec Introduction
 *
 * This is a library for RLS encoder Orbis14M with SPI comunication.
 *
 * Datasheet and SPI protocol:
 * ----> https://www.rls.si/eng/fileuploader/download/download/?d=1&file=custom%2Fupload%2FBRD01_08_EN_data_sheet.pdf
 * ----> https://www.rls.si/eng/fileuploader/download/download/?d=1&file=custom%2Fupload%2FBRD09_01_app_note_SPI.pdf
 *
 *
 * @section author Author 
 * 
 * Written by Carlos R.G. (2021/12/13)
 * Updated by 
 *
 */

#include "RLS_Orbis14M.h"

RLS_Orbis14M::RLS_Orbis14M() { spi = nullptr; }

bool RLS_Orbis14M::begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss) 
{
  if(!(spi = new SPIClass(SPI))){
    return false;
  }
  pinMode(sck, OUTPUT);
  pinMode(mosi, OUTPUT);
  pinMode(miso, INPUT);
  pinMode(ss, OUTPUT);
  digitalWrite(ss, HIGH);
  spi->begin(sck, miso, mosi, ss);
  
  return true;
}

float RLS_Orbis14M::readAngularPosition(uint8_t ratio_motor, bool clockwise_positive = true) 
{
  float angle;
  uint16_t multi_turn_counter;
  uint16_t encoder_position;
  uint8_t rx_bytes[5];

  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1)); 
  digitalWrite(spi->pinSS(), false);
  delayMicroseconds(8);
  for (uint8_t i = 0; i < sizeof(rx_bytes); i++)
  {
    rx_bytes[i] = spi->transfer(0x00);
  }
  digitalWrite(spi->pinSS(), true);
  spi->endTransaction();

  if (!bitRead(rx_bytes[3], 0)){
      Serial.println("WARNING!");  
  }
  if (!bitRead(rx_bytes[3], 1)){
    Serial.println("ERROR!!");
  }

  multi_turn_counter = (rx_bytes[0] << 8 ) | rx_bytes[1] ;
  encoder_position = (rx_bytes[2] << 6 ) |  (rx_bytes[3]>>2) ;

#if (ORBIS14M_DEBUG == 1)
  Serial.print("Multiturn counter(decimal): ");
  Serial.println(multi_turn_counter,DEC);
  Serial.print("Multiturn counter(hexadecimal): ");
  Serial.println(multi_turn_counter,HEX);

  Serial.print("Encoder position(decimal): ");
  Serial.println(encoder_position,DEC);
  Serial.print("Encoder position(hexadecimal): ");
  Serial.println(encoder_position,HEX);
#endif

  if (((float)multi_turn_counter) < 32798) 
  {
    angle = ((float)multi_turn_counter + ((float)encoder_position)/16384) * (360.0/ratio_motor); // 14 bits = 16384 values  &  360º = ratio_motor rounds 
  }
  else 
  {
    angle = ((float)(multi_turn_counter - 65535) + ((float)(encoder_position - 16384))/16384) * (360.0/ratio_motor); 
  }

  if (clockwise_positive)
  {
    angle = (-1) * angle;
  }

  return angle;
}

float RLS_Orbis14M::readLinearPosition(uint8_t ratio) 
{
  // TODO

  return 0.0;
}

int8_t RLS_Orbis14M::readStatus(void) 
{
  uint8_t status;
  uint8_t rx_bytes[6];

  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1)); 
  digitalWrite(spi->pinSS(), false);
  delayMicroseconds(8);
  rx_bytes[0] = spi->transfer(ORBIS14M_DETAILED_STATUS);
  for (uint8_t i = 1; i < sizeof(rx_bytes); i++)
  {
    rx_bytes[i] = spi->transfer(0x00);
  }
  digitalWrite(spi->pinSS(), true);
  spi->endTransaction();
  status =  rx_bytes[4];
  return status;
}

uint16_t RLS_Orbis14M::readTemperature(void) 
{
  uint16_t temp;
  uint8_t rx_bytes[7];

  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1)); 
  digitalWrite(spi->pinSS(), false);
  delayMicroseconds(8);
  rx_bytes[0] = spi->transfer(ORBIS14M_TEMP);
  for (uint8_t i = 1; i < sizeof(rx_bytes); i++)
  {
    rx_bytes[i] = spi->transfer(0x00);
  }
  digitalWrite(spi->pinSS(), true);
  spi->endTransaction();
  temp =  (rx_bytes[4] << 8 ) | rx_bytes[5];
  return temp;
}

void RLS_Orbis14M::setPositionOffsetToZero(void) 
{
  uint16_t encoder_position;
  uint8_t rx_bytes[5];

  resetSettings(); // Erase actual offset
  delay(1000);

  digitalWrite(spi->pinSS(), false);
  delayMicroseconds(8);
  for (uint8_t i = 0; i < sizeof(rx_bytes); i++)
  {
    rx_bytes[i] = spi->transfer(0x00);
  }
  digitalWrite(spi->pinSS(), true);
  spi->endTransaction();

  encoder_position = (rx_bytes[2] << 6 ) |  (rx_bytes[3]>>2) ;

  Serial.print("Encoder(decimal): ");
  Serial.println(encoder_position,DEC);
  Serial.print("Encoder pos(hexadecimal): ");
  Serial.println(encoder_position,HEX);

  uint8_t cmd[5] = {ORBIS14M_SET_ZERO_OFFSET, 0x00, 0x00, highByte(encoder_position), lowByte(encoder_position)};

  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1)); 
  unlock_();
  for (uint8_t i=0; i < sizeof(cmd); i++) {
    writeByte_(cmd[i]);
    delay(1);
  }
  spi->endTransaction();
  
  storeParams();
}

void RLS_Orbis14M::setCounterValue(uint16_t value) 
{
  value = constrain(value, 0, 65535);
  uint8_t cmd[5] = {ORBIS14M_SET_TURN_COUNTER, 0x00, 0x00, highByte(value), lowByte(value)};

  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1)); 
  unlock_();
  for (uint8_t i=0; i < sizeof(cmd); i++) {
    writeByte_(cmd[i]);
    delay(1);
  }
  spi->endTransaction();
}

void RLS_Orbis14M::storeParams(void) 
{
  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1)); 
  unlock_();
  writeByte_(ORBIS14M_SAVE_CONFIG);
  delay(1);
  spi->endTransaction();
}

void RLS_Orbis14M::resetSettings(void) 
{
  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1)); 
  unlock_();
  writeByte_(ORBIS14M_RESET_FACTORY);
  delay(1);
  spi->endTransaction();
}

void RLS_Orbis14M::selfCalibration(void) 
{
  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1)); 
  unlock_();
  writeByte_(ORBIS14M_SELF_CALIBRATION);
  delay(1);
  spi->endTransaction();
}


/*********************************************************************/

void RLS_Orbis14M::unlock_(void) 
{
  uint8_t unlock_sequence[4]={0xCD, 0xEF, 0x89, 0xAB};

  for (uint8_t i=0; i < sizeof(unlock_sequence); i++) {
    writeByte_(unlock_sequence[i]);
    delay(1);
  }
}

void RLS_Orbis14M::writeByte_(uint8_t data) 
{
  digitalWrite(spi->pinSS(), false);
  delayMicroseconds(8); // Datasheet: Time after NCS low to first SCK rising edge
  spi->transfer(data);  
  digitalWrite(spi->pinSS(), true);
}

bool RLS_Orbis14M::checkCRC_(uint8_t crc) 
{
  // TODO

  return true; 
}