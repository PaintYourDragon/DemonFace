/***************************************************************************
 This is an EXTREMELY barebones class for Adafruit LED matrix backpacks.
 It's rude code that pulls shenanigans.  Some things to be aware of:

 - This is not extensively battle-tested, and surely contains bugs.
   Honestly, I'm not even sure if the I2C retries/timeout are canon.
 - This is not a subclass of Adafruit_GFX; NO drawing primitives are present
   (must use PROGMEM bitmaps, or construct bitmap in RAM), nor is rotation
   supported (bitmap pattern must accommodate physical layout).
 - Backpack esoterica like brightness, blink rate, etc. are not handled;
   always runs full brightness, blink disabled.  Very direct stuff.
 - This code bypasses the Arduino Wire library and handles I2C (TWI)
   communication directly -- it's ATmega-specific, nonportable, and does
   some crass things (e.g. 888 KHz TWI, fastest speed supported by this
   chip) -- don't even try using Wire library at the same time!  Only TWI
   master transmit is handled; other modes are not implemented.

 Why?

 - Reduced resource use, especially RAM.  The Wire library allocates a
   good chunk for read/write buffers, and Adafruit_GFX has a bitmap buffer
   for each matrix.  And core Arduino functionality seems to get a little
   more bloaty with each release.  Mostly this code's intended for use with
   the WaveHC library, which needs more than half the MCU's RAM for loading
   two SD card blocks at a time (plus various filesystem data) -- together
   with Wire and GFX, hard limits were being exceeded, sketches failing.
 - It uses polling rather than interrupts.  This might yield slightly better
   audio quality with WaveHC playback and/or the voice changer sketches.

 This is NOT officially-supported Adafruit code, just something Phil B.
 did goofing around on his own time. No warranty, no support, no expectation
 of eternal compatibility with future Arduino releases.  Use at own risk.
 ***************************************************************************/

#include "MatrixLite.h"

// Public functions:

// Constructor
MatrixLite::MatrixLite(uint8_t a) : addr(a << 1) {
  static boolean initialized = false;
  if(!initialized) {            // First matrix declaration?
    pinMode(SDA, INPUT_PULLUP); // Enable pullups
    pinMode(SCL, INPUT_PULLUP); // on I2C pins
    TWSR = 0; TWBR = 1;         // Max supported I2C speed (888 KHz)
    initialized = true;
  }
}

// Issue startup sequence to matrix before use
void MatrixLite::begin(void) {
  twiStart();
  for(uint8_t i=0; (i<17) && twiWrite(0); i++); // Clear matrix buffer
  twiStop();
  cmd(0x21); // Oscillator on
  cmd(0xEF); // Max brightness, no blink
  cmd(0x81); // Display on
}

// Issue single-byte command to matrix.  Allows access
// to brightness & blink features if really needed.  Ugh.
void MatrixLite::cmd(uint8_t n) {
  if(twiStart()) twiWrite(n);
  twiStop();
}

// These tables compensate for the physical order in which the LED matrix
// columns are wired, allowing the easy use of bitwise values to construct
// bitmap images -- e.g. so 0b00001111 lights the 4 right-most columns.
static const uint8_t PROGMEM
  reorder8[] = { // For 8x8 matrix, reverse bit order + ROR
   0x00,0x40,0x20,0x60,0x10,0x50,0x30,0x70,0x08,0x48,0x28,0x68,0x18,0x58,0x38,
   0x78,0x04,0x44,0x24,0x64,0x14,0x54,0x34,0x74,0x0c,0x4c,0x2c,0x6c,0x1c,0x5c,
   0x3c,0x7c,0x02,0x42,0x22,0x62,0x12,0x52,0x32,0x72,0x0a,0x4a,0x2a,0x6a,0x1a,
   0x5a,0x3a,0x7a,0x06,0x46,0x26,0x66,0x16,0x56,0x36,0x76,0x0e,0x4e,0x2e,0x6e,
   0x1e,0x5e,0x3e,0x7e,0x01,0x41,0x21,0x61,0x11,0x51,0x31,0x71,0x09,0x49,0x29,
   0x69,0x19,0x59,0x39,0x79,0x05,0x45,0x25,0x65,0x15,0x55,0x35,0x75,0x0d,0x4d,
   0x2d,0x6d,0x1d,0x5d,0x3d,0x7d,0x03,0x43,0x23,0x63,0x13,0x53,0x33,0x73,0x0b,
   0x4b,0x2b,0x6b,0x1b,0x5b,0x3b,0x7b,0x07,0x47,0x27,0x67,0x17,0x57,0x37,0x77,
   0x0f,0x4f,0x2f,0x6f,0x1f,0x5f,0x3f,0x7f,0x80,0xc0,0xa0,0xe0,0x90,0xd0,0xb0,
   0xf0,0x88,0xc8,0xa8,0xe8,0x98,0xd8,0xb8,0xf8,0x84,0xc4,0xa4,0xe4,0x94,0xd4,
   0xb4,0xf4,0x8c,0xcc,0xac,0xec,0x9c,0xdc,0xbc,0xfc,0x82,0xc2,0xa2,0xe2,0x92,
   0xd2,0xb2,0xf2,0x8a,0xca,0xaa,0xea,0x9a,0xda,0xba,0xfa,0x86,0xc6,0xa6,0xe6,
   0x96,0xd6,0xb6,0xf6,0x8e,0xce,0xae,0xee,0x9e,0xde,0xbe,0xfe,0x81,0xc1,0xa1,
   0xe1,0x91,0xd1,0xb1,0xf1,0x89,0xc9,0xa9,0xe9,0x99,0xd9,0xb9,0xf9,0x85,0xc5,
   0xa5,0xe5,0x95,0xd5,0xb5,0xf5,0x8d,0xcd,0xad,0xed,0x9d,0xdd,0xbd,0xfd,0x83,
   0xc3,0xa3,0xe3,0x93,0xd3,0xb3,0xf3,0x8b,0xcb,0xab,0xeb,0x9b,0xdb,0xbb,0xfb,
   0x87,0xc7,0xa7,0xe7,0x97,0xd7,0xb7,0xf7,0x8f,0xcf,0xaf,0xef,0x9f,0xdf,0xbf,
   0xff },
  reorder16[] = { // For 16x8 matrix (or bicolor), reverse bit order (no ROR)
   0x00,0x80,0x40,0xc0,0x20,0xa0,0x60,0xe0,0x10,0x90,0x50,0xd0,0x30,0xb0,0x70,
   0xf0,0x08,0x88,0x48,0xc8,0x28,0xa8,0x68,0xe8,0x18,0x98,0x58,0xd8,0x38,0xb8,
   0x78,0xf8,0x04,0x84,0x44,0xc4,0x24,0xa4,0x64,0xe4,0x14,0x94,0x54,0xd4,0x34,
   0xb4,0x74,0xf4,0x0c,0x8c,0x4c,0xcc,0x2c,0xac,0x6c,0xec,0x1c,0x9c,0x5c,0xdc,
   0x3c,0xbc,0x7c,0xfc,0x02,0x82,0x42,0xc2,0x22,0xa2,0x62,0xe2,0x12,0x92,0x52,
   0xd2,0x32,0xb2,0x72,0xf2,0x0a,0x8a,0x4a,0xca,0x2a,0xaa,0x6a,0xea,0x1a,0x9a,
   0x5a,0xda,0x3a,0xba,0x7a,0xfa,0x06,0x86,0x46,0xc6,0x26,0xa6,0x66,0xe6,0x16,
   0x96,0x56,0xd6,0x36,0xb6,0x76,0xf6,0x0e,0x8e,0x4e,0xce,0x2e,0xae,0x6e,0xee,
   0x1e,0x9e,0x5e,0xde,0x3e,0xbe,0x7e,0xfe,0x01,0x81,0x41,0xc1,0x21,0xa1,0x61,
   0xe1,0x11,0x91,0x51,0xd1,0x31,0xb1,0x71,0xf1,0x09,0x89,0x49,0xc9,0x29,0xa9,
   0x69,0xe9,0x19,0x99,0x59,0xd9,0x39,0xb9,0x79,0xf9,0x05,0x85,0x45,0xc5,0x25,
   0xa5,0x65,0xe5,0x15,0x95,0x55,0xd5,0x35,0xb5,0x75,0xf5,0x0d,0x8d,0x4d,0xcd,
   0x2d,0xad,0x6d,0xed,0x1d,0x9d,0x5d,0xdd,0x3d,0xbd,0x7d,0xfd,0x03,0x83,0x43,
   0xc3,0x23,0xa3,0x63,0xe3,0x13,0x93,0x53,0xd3,0x33,0xb3,0x73,0xf3,0x0b,0x8b,
   0x4b,0xcb,0x2b,0xab,0x6b,0xeb,0x1b,0x9b,0x5b,0xdb,0x3b,0xbb,0x7b,0xfb,0x07,
   0x87,0x47,0xc7,0x27,0xa7,0x67,0xe7,0x17,0x97,0x57,0xd7,0x37,0xb7,0x77,0xf7,
   0x0f,0x8f,0x4f,0xcf,0x2f,0xaf,0x6f,0xef,0x1f,0x9f,0x5f,0xdf,0x3f,0xbf,0x7f,
   0xff };

// Random idea: might modify these functions to use raw TWI ops rather than
// calling Start/Write/Stop (see private functions later)...idea being to
// queue up next value in the idle time before polling TWI status, similar
// to SPI pipelining.  Low priority insane optimization; might shave a few
// microseconds/update at best, code would get very ugly with the retry
// loops & such.

// Update 8x8 matrix from bitmap in RAM
void MatrixLite::show(uint8_t *data) {
  if(twiStart()) {
    // Matrix driver expects write address followed by data (typically
    // zero followed by 16 bytes for the full display buffer).  8x8 matrix
    // only uses odd-numbered bytes; even bytes are ignored.  Little trick
    // here is to alternate eight 0's and eight data bytes -- the first 0
    // is interpreted as the start address, and then only 15 add'l bytes
    // are sent, last byte can be dropped since it's unused anyway.
    for(uint8_t i=0; twiWrite(0) &&
      twiWrite(pgm_read_byte(&reorder8[data[i++]])) && (i < 8); );
  }
  twiStop();
}

// Update 8x8 matrix from bitmap in PROGMEM
void MatrixLite::show(const uint8_t *data) {
  if(twiStart()) {
    for(uint8_t i=0; twiWrite(0) && twiWrite(
      pgm_read_byte(&reorder8[pgm_read_byte(&data[i++])])) && (i < 8); );
  }
  twiStop();
}

// Update 16x8 mono or 8x8 bicolor matrix from bitmap in RAM
void MatrixLite::show(uint16_t *data) {
  if(twiStart() && twiWrite(0)) {
    uint16_t n;
    for(uint8_t i=0; i<8; i++) {
      n = *data++;
      twiWrite(pgm_read_byte(&reorder16[n >> 8  ]));
      twiWrite(pgm_read_byte(&reorder16[n & 0xFF]));
    }
  }
  twiStop();
}

// Update 16x8 mono or 8x8 bicolor matrix from bitmap in PROGMEM
void MatrixLite::show(const uint16_t *data) {
  if(twiStart() && twiWrite(0)) {
    uint16_t n;
    for(uint8_t i=0; i<8; i++) {
      n = pgm_read_word(data++);
      twiWrite(pgm_read_byte(&reorder16[n >> 8  ]));
      twiWrite(pgm_read_byte(&reorder16[n & 0xFF]));
    }
  }
  twiStop();
}

// Private functions:

#define TWSR_MASK    0xF8 // TWI status register values
#define TWI_START    0x08
#define TWI_SLAW_ACK 0x18
#define TWI_DATA_ACK 0x28
#define TWI_RETRIES     4
#define TWI_TIMEOUT   200

// Begin TWI transmission; returns true on success, false otherwise
boolean MatrixLite::twiStart(void) {
  uint8_t r, t;

  for(r=0; r<TWI_RETRIES; r++) {
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);                // Send START,
    for(t=0; (!(TWCR & _BV(TWINT))) && (t++ < TWI_TIMEOUT); ); // Await flag
    if((TWSR & TWSR_MASK) == TWI_START) break;                 // START OK
  }

  if(r <= TWI_RETRIES) { // START OK
    for(uint8_t r=0; r<TWI_RETRIES; r++) {
      TWDR = addr;                   // TWI Address + write flag (0)
      TWCR = _BV(TWINT) | _BV(TWEN); // Transmit address
      for(t=0; (!(TWCR & _BV(TWINT))) && (t++ < TWI_TIMEOUT); );
      if((TWSR & TWSR_MASK) == TWI_SLAW_ACK) return true; // ACK (OK)
    }
  }

  return false; // FAIL, timeout or repeated NAKs
}

// Send byte over TWI (after Start); true on success, false otherwise
boolean MatrixLite::twiWrite(uint8_t n) {
  uint8_t r, t;
  for(r=0; r<TWI_RETRIES; r++) {
    TWDR = n;                      // Data to send
    TWCR = _BV(TWINT) | _BV(TWEN); // Start data transmission
    for(t=0; (!(TWCR & _BV(TWINT))) && (t++ < TWI_TIMEOUT); );
    if((TWSR & TWSR_MASK) == TWI_DATA_ACK) return true; // ACK (OK)
  }
  return false;                    // FAIL, repeated NAKs
}

// End TWI transmission (no return value)
void MatrixLite::twiStop(void) {
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO); // Transmit STOP
  for(uint8_t t=0; (!(TWCR & _BV(TWINT))) && (t++ < TWI_TIMEOUT); );
}
