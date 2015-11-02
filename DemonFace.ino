/***************************************************************************
 This is a HEAVILY MUTATED derivative of the adavoice_face sketch.
 It is NOT a drop-in replacement for that code.  There are changes in the
 matrix layout and the bitmaps that are specific to Phil's mask, plus a
 bunch of experimental (and potentially buggy & nonportable) things.

 This is NOT officially-supported Adafruit code, just something Phil B.
 did goofing around on his own time. No warranty, no support, no expectation
 of eternal compatibility with future Arduino releases.  Use at own risk.

 Additionally, this code does unspeakably horrible things that would get
 you an "F" in CompSci or EE...do NOT look to this code as a role model for
 anything.  ADC is overclocked, I2C is run at a non-standard bitrate, the
 Adafruit_LEDBackpack library is bypassed to address the LED matrices
 directly.  See notes in MatrixLite.cpp.  Most of these are feeble attempts
 to improve the audio quality, either through more uniform interrupt timing
 or minimizing the time spent in I2C transmissions (which may cause line
 interference with analog input from mic).
 ***************************************************************************/

#include <WaveHC.h>
#include <WaveUtil.h>
#include "MatrixLite.h"
#include "bitmaps.h"

//#define DEBUG // Print stuff to serial port
#define GRAINLEN         11.1     // Duration of audio grain, in milliseconds
#define FACE_FPS         50       // Face anim. rate (determines blink speed)
#define ADC_CHANNEL      0        // Microphone on Analog pin 0
#define PITCH_ANALOG_REF EXTERNAL // Pitch knob: EXTERNAL=3.3V, DEFAULT=5V
#define PITCH_2X_LIMIT   700      // Max pitch where 2X sampling still works

#define DAC_CS_PORT      PORTD    // Wave shield DAC: digital pins 2,3,4,5
#define DAC_CS           PORTD2
#define DAC_CLK_PORT     PORTD
#define DAC_CLK          PORTD3
#define DAC_DI_PORT      PORTD
#define DAC_DI           PORTD4
#define DAC_LATCH_PORT   PORTD
#define DAC_LATCH        PORTD5

SdReader  card; // Global SD filesystem stuff,
FatVolume vol;  // used when playing sounds.
FatReader root;

// WaveHC didn't declare it's working buffers private or static,
// so we can be sneaky and borrow the same RAM for audio sampling!
extern volatile uint8_t buffer1[PLAYBUFFLEN], // Audio sample LSB
                        buffer2[PLAYBUFFLEN]; // Audio sample MSB
volatile uint8_t       *xf1, *xf2;            // Cross-fade LSB/MBS bufs
#define XFADEBITS    4                        // 2-5 bits (4-32)
#define XFADE       (1 << XFADEBITS)          // # of cross-fade samples
#define MAX_SAMPLES (PLAYBUFFLEN - XFADE)     // Audio sampling limit

volatile int16_t  in  = 0,             // Index of last audio sample stored
                  out = 0;             // Index of next sample out
volatile uint8_t  xf  = 0;             // Index of last xfade sample stored
uint16_t          nSamples;            // Audio sample buffer size
uint8_t           adc_save;            // Save/restore ADC config
volatile uint8_t  oldsum      = 0;     // For averaging all of the
volatile uint32_t newsum      = 0L;    // audio samples in buffers

uint16_t          faceLimit   = 1;     // # ADC samples per face update
volatile uint16_t faceCounter = 1;     // Countdown to next update
volatile boolean  updateFace  = false; // true when update due
volatile uint8_t  breathPos   = 0;     // Index into breath PWM table

uint8_t blinkCountdown = 100, // Countdown to next blink (in frames)
        gazeCountdown  =  75, // Countdown to next eye movement
        gazeFrames     =  50; // Duration of eye movement (smaller = faster)
int8_t  eyeX = 3, eyeY = 3,   // Current eye position
        newX = 3, newY = 3,   // Next eye position
        dX   = 0, dY   = 0;   // Distance from prior to new position

#define MATRIX_EYES         0 // Indices in matrix[] array below
#define MATRIX_MOUTH_LEFT   1
#define MATRIX_MOUTH_MIDDLE 2
#define MATRIX_MOUTH_RIGHT  3
MatrixLite matrix[4] = { MatrixLite(0x70), MatrixLite(0x71),
                         MatrixLite(0x72), MatrixLite(0x73) };

//-------------------------------------------------------------------------

void setup() {
  FatReader file; // For startup sound
  WaveHC    wave; // "
  uint8_t   i;

#ifdef DEBUG
  Serial.begin(9600);
  Serial.println(F("HELLO!"));
#endif

  randomSeed(analogRead(A3)); // Seed random() from unused analog pin
  pinMode(A2, INPUT_PULLUP);  // Pullup on voice pitch button (PINC2)

  for(i=0; i<4; i++) {
    matrix[i].begin();          // Init each matrix
    matrix[i].show(bootImg[i]); // Show boot face
  }

  // Timer/Counter 0 is used for the PWM breathing effect on pin 6.  This
  // timer (and associated interrupt) normally provide delay(), millis()
  // and related functions -- they're unavailable and unused in the rest of
  // this code.  No interrupt means slightly more uniform audio sampling.
  pinMode(6, OUTPUT);
  TIMSK0 = 0; // Timer/Counter 0 interrupts off
  //TCCR0B = _BV(CS00); // 1:1 prescale;
  TCCR0B = _BV(CS02); // 1:256 prescale; audio interference less noticeable
  TCCR0A = _BV(COM0A1) | _BV(WGM01) | _BV(WGM00); // Fast PWM, non-inverting
  OCR0A  = pgm_read_byte(breathTable);

  // The WaveHC library normally initializes the DAC pins...but only after
  // an SD card is detected and a valid file is passed.  Need to init the
  // pins manually here so that voice FX works even without a card.
  pinMode(2, OUTPUT);         // Chip select
  pinMode(3, OUTPUT);         // Serial clock
  pinMode(4, OUTPUT);         // Serial data
  pinMode(5, OUTPUT);         // Latch
  DAC_CS_PORT |= _BV(DAC_CS); // Set chip select high

  // Init SD library.  Errors are displayed but NOT regarded as fatal --
  // program will continue with voice FX regardless!
  if(!card.init())             Serial.println(F("Card init. failed!"));
  else if(!vol.init(card))     Serial.println(F("No partition!"));
  else if(!root.openRoot(vol)) Serial.println(F("Couldn't open dir"));

  // Play startup sound
  if(file.open(root, "startup.wav")) {
    if(wave.create(file)) {
      wave.play();
      while(wave.isplaying);
    }
    file.close();
  }

  startPitchShift();
}

//-------------------------------------------------------------------------

uint8_t prevMouthPos = 0xFF,  // Prior frame positions
        pX = 0xFF, pY = 0xFF; // Prior pupil X/Y (redraw eye only on change)

void loop() {
  uint8_t blinkPos = 0, // Current frame # of eye blinkyness
          xx, yy,       // Current pupil X/Y position for this frame
          mouthPos, bits;

  while(!updateFace) { // While awaiting face update from interrupt...
    if(!(PINC & _BV(PINC2))) {     // Button pressed? (A2 pulled LOW)
      stopPitchShift();            // Stop voice
      while(!(PINC & _BV(PINC2))); // Wait for release
      startPitchShift();           // Resume (pitch reading taken here)
    }
  }
  updateFace = false;  // Reset flag

  // Update breathing PWM
  OCR0A = pgm_read_byte(&breathTable[breathPos]);
  // if(++breathPos >= sizeof(breathTable)) breathPos = 0;
  breathPos++; // Is 8-bit type and table is 256 elements

  // When counting down to next blink, eye is in position 0 (fully open).
  // On the last few counts (during blink), look up bitmap index in table.
  if(--blinkCountdown < sizeof(blinkIndex)) { // Currently blinking?
    blinkPos = pgm_read_byte(&blinkIndex[blinkCountdown]);
    pX = 0xFF; // Force eye redraw
    if(!blinkCountdown) blinkCountdown = random(6, 160);
  }

  // Periodically, the pupil moves to a new position.  This is handled
  // similarly to the blink -- gazeCountdown is the total time (including
  // holding current pos.) until next position is reached, gazeFrames is
  // the actual in-motion time at the end of this countdown.
  if(--gazeCountdown <= gazeFrames) {
    if(gazeCountdown) {
      // Eyes are in motion - draw pupil at interim position
      xx = newX - dX * gazeCountdown / gazeFrames;
      yy = newY - dY * gazeCountdown / gazeFrames;
    } else { // Last frame
      eyeX = xx = newX; // Store new X/Y
      eyeY = yy = newY;
      do { // Pick random X/Y until one is within acceptable landing zone
        newX = random(1,7); newY = random(1,7);  // 1-6
      } while(!((pgm_read_byte(&landingImg[newY]) & (0x80 >> newX))));
      dX            = newX - eyeX;             // Horizontal distance to move
      dY            = newY - eyeY;             // Vertical distance to move
      gazeFrames    = random(3, 15);           // Duration of eye movement
      gazeCountdown = random(gazeFrames, 100); // Count to end of next move
    }
  } else { // Eye in steady position; no motion.
    xx = eyeX;
    yy = eyeY;
  }

  // Need to know mouth position before rendering eye; tertiary motions.
  mouthPos  = oldsum / 24;       // 0-10, yields better movement
  if(mouthPos > 7) mouthPos = 7; // Clip pos to 7 (max open bitmap)

  // Redraw eye only if pupil has moved from prior frame (actions such as
  // blinking intentionally corrupt the prior position to force redraw).
  // Updating only when there's an actual change reduces the amount of I2C
  // traffic, hopefully reducing audio interference.
  if((xx != pX) || (yy != pY)) {
    uint8_t bitmap[8];
    int8_t  lidPos;

    pX = xx; // Save positions now,
    pY = yy; // values are fiddled below

    // In addition to the normal eye blink frames, there are some
    // secondary/tertiary motions that occur to look more 'alive'.
    // The upper eyelid may drop slightly to track the position of the
    // pupil; eye isn't always fully open between blinks.  Drop is +1
    // pixel when talking loudly to approximate squint.
    lidPos = xx + yy - 4;
    if(lidPos < 0)      lidPos = 0;
    else if(lidPos > 6) lidPos = 6;

    // Eye image is composited into bitmap[] from the current blinkPos
    // image, the extra pupil-tracking eyelid image, and pupil bitmap
    // image (all in PROGMEM).
    yy = 3 - yy; // Offset pupil center to 7x7 bitmap top
    for(uint8_t i=0; i<8; i++, yy++) {
      bits =  pgm_read_byte(blinkImg[blinkPos] + i) & // Base eye blink
             ~pgm_read_byte(lidImg[lidPos]     + i);  // Mask out extra lid
      if((yy >= 0) && (yy <= 6)) {                    // Overlap Pupil bitmap?
	bits &= ~((uint16_t)pgm_read_byte(pupilImg + yy) << 4) >> xx;
      }
      bitmap[i] = bits;
    }

    // Only one eye needs to be drawn.  Because the two eye matrices share
    // the same address, the same data will be received by both.
    matrix[MATRIX_EYES].show(bitmap);
  }

  if(mouthPos != prevMouthPos) {
    for(xx=0; xx<3; xx++) {
      matrix[MATRIX_MOUTH_LEFT+xx].show(mouthImg[mouthPos][xx]);
    }
    prevMouthPos = mouthPos;
  }
}

static void startPitchShift(void) {

  analogReference(PITCH_ANALOG_REF);     // Ref for reading pitch dial
  for(volatile uint16_t i=10000; --i; ); // Settle voltage 1+ ms (no delay())

  int pitch = analogRead(1);
#ifdef DEBUG
  Serial.print(F("Pitch: "));
  Serial.println(pitch);
  Serial.print(F("Shifting "));
  Serial.print((pitch < 512) ? F("down") : F("up"));
  Serial.print(F(", "));
  Serial.write((pitch < PITCH_2X_LIMIT) ? '2' : '1');
  Serial.println(F("X sampling rate"));
#endif

  // 2 octave (+/-1) audio taper; 0 to 1023 in, 0.5 to 2.0 out (1.0 = normal)
  float taper = 0.5 + pow(((float)pitch / 1023.0), 1.58496) * 1.5;
  // Configure Timer/Counter 1 for playback interrupt
  TCCR1A = _BV(WGM11) | _BV(WGM10);             // Fast PWM mode
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // No prescale
  // Trying a thing here.  At lower pitches, playback interrupt is less
  // frequent.  With more CPU cycles thus free, we can try using higher
  // sampling and playback rates, possibly yielding better sound quality
  // (technically overclocking the ADC at the faster rate, but in practice
  // seems to work OK).  The threshold for selecting the faster rates
  // does not need to be at 512; this will have to be found empirically,
  // whatever limit allows both interrupts to operate at the higher rates.
  if(pitch < PITCH_2X_LIMIT) { // Use 2X sampling rate
    // nSamples is function of sampling rate (not playback rate).
    // 250K is ADC clock / 13 is number of ADC cycles/sample / 1K to ms
    nSamples  = (int)(((250000.0 / 13.0) / 1000.0) * GRAINLEN + 0.5);
    // Face redrawn every 'faceLimit' ADC sample to approximate FACE_FPS:
    faceLimit = (uint16_t)((250000.0 / 13.0) / (float)FACE_FPS + 0.5);
    // Playback interval derives from ADC rate, scaled by taper
    OCR1A     = (uint16_t)(((float)F_CPU / ((250000.0 / 13.0) * taper)) + 0.5);
  } else { // Use 1X sampling rate (125K ADC clock)
    nSamples  = (int)(((125000.0 / 13.0) / 1000.0) * GRAINLEN + 0.5);
    faceLimit = (uint16_t)((125000.0 / 13.0) / (float)FACE_FPS + 0.5);
    OCR1A     = (uint16_t)(((float)F_CPU / ((125000.0 / 13.0) * taper)) + 0.5);
  }
  if(nSamples > MAX_SAMPLES)      nSamples = MAX_SAMPLES;
  else if(nSamples < (XFADE * 2)) nSamples = XFADE * 2;
#ifdef DEBUG
  Serial.print(F("Taper: "));
  Serial.println(taper);
  Serial.print(F("nSamples: "));
  Serial.println(nSamples);
  Serial.print(F("faceLimit: "));
  Serial.println(faceLimit);
  Serial.print(F("OCR1A: "));
  Serial.println(OCR1A);
#endif

  memset((void *)buffer1, 0, nSamples + XFADE); // Clear sample buffers
  memset((void *)buffer2, 2, nSamples + XFADE); // (set all samples to 512)
  in = out = xf = 0;
  faceCounter = faceLimit;
  xf1 = &buffer1[nSamples]; // Direct pointers to
  xf2 = &buffer2[nSamples]; // cross-fade buffer space

  analogReference(EXTERNAL);             // 3.3V to AREF
  for(volatile uint16_t i=10000; --i; ); // Settle voltage 1+ ms (no delay())
  adc_save = ADCSRA;                     // Save ADC config for restore later

  // Start up ADC in free-run mode for audio sampling:
  DIDR0 |= _BV(ADC0D);  // Disable digital input buffer on ADC0
  ADMUX  = ADC_CHANNEL; // Channel sel, right-adj, AREF to 3.3V regulator
  ADCSRB = 0;           // Free-run mode
  ADCSRA = (pitch < PITCH_2X_LIMIT) ?
    // 250 KHz ADC rate:
    _BV(ADEN)  | // Enable ADC
    _BV(ADSC)  | // Start conversions
    _BV(ADATE) | // Auto-trigger enable
    _BV(ADIE)  | // Interrupt enable
    _BV(ADPS2) | // 64:1 prescale yields 250 KHz ADC clock.
    _BV(ADPS1) : // 13 cycles/conversion = ~19230 Hz
    // 125 KHz ADC rate:
    _BV(ADEN)  | // Enable ADC
    _BV(ADSC)  | // Start conversions
    _BV(ADATE) | // Auto-trigger enable
    _BV(ADIE)  | // Interrupt enable
    _BV(ADPS2) | // 128:1 prescale yields 125 KHz ADC clock
    _BV(ADPS1) | // 13 cycles/conversion = ~9615 Hz
    _BV(ADPS0);

  // WaveHC uses Timer1 compare match interrupt; we can still use overflow!
  TIMSK1 |= _BV(TOIE1); // Enable overflow interrupt (playback)
}

static void stopPitchShift(void) {
  TIMSK1 &= ~_BV(TOIE1); // Stop playback interrupt
  ADCSRA  = adc_save;    // Restore ADC control/status reg (interrupt off)
  (void)analogRead(1);   // Discard first ADC reading (auto-trigger residue)
}

//-------------------------------------------------------------------------

ISR(ADC_vect) { // Sampling interrupt; ADC conversion complete

  if(++in >= nSamples) in = 0; // Counter/rollover for sample buffer
  if(++xf >= XFADE)    xf = 0; // Counter/rollover for xfade buf

  // Save old sample from 'in' position to xfade buffer:
  xf1[xf] = buffer1[in]; // LSB
  xf2[xf] = buffer2[in]; // MSB

  // Store new value in sample buffers:
  buffer1[in] = ADCL; // MUST read ADCL first! (LSB)
  buffer2[in] = ADCH; // MSB

  if(!in) { // If circular buffer just rolled over above...
    oldsum = (uint8_t)((newsum / nSamples) >> 1); // Average & scale 0-255
    newsum = 0L;                                  // Clear for next average
  } // (this is done here to minimize playback interrupt off time)

  // Add sample magnitude to newsum for averaging later
  newsum += abs((((int)buffer2[in] << 8) | buffer1[in]) - 512);

  if(--faceCounter == 0) {   // Countdown samples to next face update
    faceCounter = faceLimit; // Reset counter
    updateFace  = true;      // Set flag to signal loop() to resume
  }
}

ISR(TIMER1_OVF_vect) { // Playback interrupt
  uint16_t s;
  uint8_t  w, inv, hi, lo, bit, xi;

  if(++out >= nSamples) out = 0; // Counter/rollover for sample buffer

  // Cross-fade around circular buffer 'seam' to avoid clicks.
  // Some duplicitous code here, conditions sorted for probability.
  if(out > in) {
    if(out <= (in + nSamples - XFADE)) {
      // Input/output/xfade don't coincide -- use sample directly
      hi  = (buffer2[out] << 2) | (buffer1[out] >> 6); // Expand 10-bit data
      lo  = (buffer1[out] << 2) |  buffer2[out];       // to 12 bits
    } else {
      // In cross-fade area near end of buffer
      w   = in + nSamples - out; // Weight of sample (1 to (XFADE-1))
      inv = XFADE - w;           // Weight of xfade (1 to (XFADE-1))
      xi  = (xf + inv) & (XFADE-1);
      s   = ((buffer2[out] << 8) | buffer1[out]) * w +
            ((xf2[xi]      << 8) | xf1[xi]     ) * inv;
      hi  = s >> (XFADEBITS + 6); // Shift >12 bit result
      lo  = s >> (XFADEBITS - 2); // down to 12 bits
    }
  } else if(out != in) {
    if(out <= (in - XFADE)) {
      // Input/output/xfade don't coincide -- use sample directly
      hi  = (buffer2[out] << 2) | (buffer1[out] >> 6); // Expand 10-bit data
      lo  = (buffer1[out] << 2) |  buffer2[out];       // to 12 bits
    } else {
      // In cross-fade area early- to mid-buffer
      w   = in - out;  // Weight of sample (1 to (XFADE-1))
      inv = XFADE - w; // Weight of xfade  (1 to (XFADE-1))
      xi  = (xf + inv) & (XFADE-1);   // Index into xfade buf
      s   = ((buffer2[out] << 8) | buffer1[out]) * w +  // Weighted sample
            ((xf2[xi]      << 8) | xf1[xi]     ) * inv; // Weighted xfade
      hi  = s >> (XFADEBITS + 6); // Shift >12 bit result
      lo  = s >> (XFADEBITS - 2); // down to 12 bits
    }
  } else {
    // Sample positions coincide -- use sample most recently moved to xfade
    hi = (xf2[xf] << 2) | (xf1[xf] >> 6); // Expand 10-bit data
    lo = (xf1[xf] << 2) |  xf2[xf];       // to 12 bits
  }

  // Might be possible to tweak 'hi' and 'lo' at this point to achieve
  // different voice modulations -- robot effect, etc.?

  DAC_CS_PORT &= ~_BV(DAC_CS); // Select DAC
  // Clock out 4 bits DAC config (not in loop because it's constant)
  DAC_DI_PORT  &= ~_BV(DAC_DI); // 0 = Select DAC A, unbuffered
  DAC_CLK_PORT |=  _BV(DAC_CLK); DAC_CLK_PORT &= ~_BV(DAC_CLK);
  DAC_CLK_PORT |=  _BV(DAC_CLK); DAC_CLK_PORT &= ~_BV(DAC_CLK);
  DAC_DI_PORT  |=  _BV(DAC_DI); // 1X gain, enable = 1
  DAC_CLK_PORT |=  _BV(DAC_CLK); DAC_CLK_PORT &= ~_BV(DAC_CLK);
  DAC_CLK_PORT |=  _BV(DAC_CLK); DAC_CLK_PORT &= ~_BV(DAC_CLK);
  for(bit=0x08; bit; bit>>=1) { // Clock out first 4 bits of data
    if(hi & bit) DAC_DI_PORT |=  _BV(DAC_DI);
    else         DAC_DI_PORT &= ~_BV(DAC_DI);
    DAC_CLK_PORT |=  _BV(DAC_CLK); DAC_CLK_PORT &= ~_BV(DAC_CLK);
  }
  for(bit=0x80; bit; bit>>=1) { // Clock out last 8 bits of data
    if(lo & bit) DAC_DI_PORT |=  _BV(DAC_DI);
    else         DAC_DI_PORT &= ~_BV(DAC_DI);
    DAC_CLK_PORT |=  _BV(DAC_CLK); DAC_CLK_PORT &= ~_BV(DAC_CLK);
  }
  DAC_CS_PORT |= _BV(DAC_CS); // Unselect DAC
}
