// This is nasty.  See MatrixLite.cpp for explanation.

#ifndef _MATRIXLITE_H_
#define _MATRIXLITE_H_
#include <Arduino.h>

class MatrixLite {
 public:
  MatrixLite(uint8_t a);           // Constructor; pass I2C address
  void begin(void),                // Initialize matrix before use
       show(uint8_t        *data), // Update 8x8 matrix from RAM bitmap
       show(const uint8_t  *data), // Update 8x8 matrix from PROGMEM bitmap
       show(uint16_t       *data), // Update 16x8 matrix from RAM bitmap
       show(const uint16_t *data), // Update 16x8 matrix from PROGMEM bitmap
       cmd(uint8_t n);             // Start+Write(n)+Stop
       // cmd is public so matrix features (brightness, etc.)
       // can be accessed if really neeed.
 private:
  uint8_t addr;                    // 'Raw' TWI addr (address << 1)
  boolean twiStart(void),          // Begin transmission
          twiWrite(uint8_t n);     // Issue byte
  void    twiStop(void);           // End transmission
};

#endif // _MATRIXLITE_H_
