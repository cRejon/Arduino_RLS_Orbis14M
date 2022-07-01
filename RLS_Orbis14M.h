/*!
 * @file RSL_Orbis14M.h
 *
 * This is a library for RLS encoder Orbis14M with SPI comunication.
 *
 * Datasheet and SPI protocol:
 * ----> https://www.rls.si/eng/fileuploader/download/download/?d=1&file=custom%2Fupload%2FBRD01_08_EN_data_sheet.pdf
 * ----> https://www.rls.si/eng/fileuploader/download/download/?d=1&file=custom%2Fupload%2FBRD09_01_app_note_SPI.pdf
 *
 *
 * Written by Carlos R.G. (2021/12/13)
 * 
 */

#ifndef RLS_ORBIS14M_H
#define RLS_ORBIS14M_H

#include <SPI.h>
#include <Arduino.h>

#define ORBIS14M_DEBUG 1                    //!< Debug mode
// Request command byes
#define ORBIS14M_DETAILED_STATUS 0x64       //!< Detailed status request
#define ORBIS14M_TEMP 0x74                  //!< Temperature request
#define ORBIS14M_SERIAL_NUM 0x76            //!< Serial number request
// Programming command bytes
#define ORBIS14M_SET_ZERO_OFFSET 0x5A       //!< Setting zero offset
#define ORBIS14M_SET_TURN_COUNTER 0x4D      //!< Multiturn counter setting
#define ORBIS14M_SAVE_CONFIG 0x63           //!< Configuration parameters save
#define ORBIS14M_RESET_FACTORY 0x72         //!< Reset to factory defaults
#define ORBIS14M_SELF_CALIBRATION 0x41      //!< Triggering self-calibration

#define SPI VSPI                            //!< VSPI or HSPI


static const int spiClk = 1000000; // 1 MHz

/*!
 * @brief Main Orbis14M class
 */
class RLS_Orbis14M {
public:
  RLS_Orbis14M();
  /*!
   * @brief Starts GPIOs and SPI connection
   * @param sck SPI clock pin
   * @param miso SPI MISO pin
   * @param mosi SPI MOSI pin
   * @param ss SPI chip select pin
   * @return Returns true if successful
   */
  bool begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss);
  /*!
   * @brief Gets the angular position
   * @param ratio_motor Motor reduction ratio
   * @param clockwise_positive Direction of rotation for positive angle
   * @return Returns the angle in degrees
   */
  float readAngularPosition(uint8_t ratio_motor, bool clockwise_positive);
  /*!
   * @brief Gets the linear position
   * @param ratio Ratio between distance and angle in mm/degrees
   * @return Returns the distance in mm
   */
  float readLinearPosition(uint8_t ratio);
  /*!
   * @brief Gets the status from encoder
   * @return Returns the detailed status (see datasheet) 
   */
  int8_t readStatus(void);
  /*!
   * @brief Reads temperature of the readhead  
   * @return Returns temperature in °C multiplied by 10
   */
  uint16_t readTemperature(void);
  /*!
   * @brief First transfer zero position offset into RAM. Second, send the command to store 
   *  this value into non-volatile memory.  
   */
  void setPositionOffsetToZero(void);
  /*!
   * @brief Sets the multiturn counter value
   * @param value Value for multiturn counter (0 <= value <= 65535)
   */
  void setCounterValue(uint16_t value);
  /*!
   * @brief The self-calibration function eliminates eccentricity-caused error
   */
  void selfCalibration(void);
  /*!
   * @brief Save configuration parameters to non-volatile memory
   */
  void storeParams(void);
  /*!
   * @brief Reset zero position offset to 0 and self-calibration parameters to factory defaults
   */
  void resetSettings(void);

private:
  void unlock_(void);
  void writeByte_(uint8_t data);
  bool checkCRC_(uint8_t crc);

  SPIClass * spi;

};

#endif //  RLS_ORBIS14M_H