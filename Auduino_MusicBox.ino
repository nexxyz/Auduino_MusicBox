// Auduino MusicBox by nexxyz
// This code is very dirty and just hacked together for my private use
// If you want to use it - feel free to!
// I have added (to the original Auduino):
// * a simple code-sequencer where you can list melodies by note offset
// * an additional encoder to step through melodies
// * an additional button (can be combined with the encoder)
// ** short press selects melody
// ** long press selects mode
// * a mode where you step through the melodies' notes one by one
// * a music-box-like mode where you can "wind up" the melody, and playback gets slower when it reaches the end
// * a mode where each note is played automatically
// * a very basic volume decay/release, configurable by an additional potentiometer
// * an additional encoder to increase/decrease the tempo of the sequencer - untested because I still need to buy an encoder :-)
//
// Uses AdaEncoder - https://code.google.com/p/adaencoder/ (depends on https://code.google.com/p/oopinchangeint/)
// Uses ClickButton - https://code.google.com/p/clickbutton/
// Uses Timer - http://playground.arduino.cc/Code/Timer
//
// Original:
// Auduino, the Lo-Fi granular synthesiser
//
// by Peter Knight, Tinker.it http://tinker.it
//
// Help:      http://code.google.com/p/tinkerit/wiki/Auduino
// More help: http://groups.google.com/group/auduino
//
// Analog in 0: Grain 1 pitch
// Analog in 1: Grain 2 decay
// Analog in 2: Grain 1 decay
// Analog in 3: Grain 2 pitch
// Analog in 4: Grain repetition frequency
//
// Digital 3: Audio out (Digital 11 on ATmega8)
//
// Changelog:
// 19 Nov 2008: Added support for ATmega8 boards
// 21 Mar 2009: Added support for ATmega328 boards
// 7 Apr 2009: Fixed interrupt vector for ATmega328 boards
// 8 Apr 2009: Added support for ATmega1280 boards (Arduino Mega)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <ClickButton.h>
#include <ooPinChangeInt.h>
#include <AdaEncoder.h>
#include <Event.h>
#include <Timer.h>

uint16_t syncPhaseAcc;
uint16_t syncPhaseInc;
uint16_t grainPhaseAcc;
uint16_t grainPhaseInc;
uint16_t grainAmp;
uint8_t grainDecay;
uint16_t grain2PhaseAcc;
uint16_t grain2PhaseInc;
uint16_t grain2Amp;
uint8_t grain2Decay;

// Map Analogue channels
#define SYNC_CONTROL         (4)
#define GRAIN_FREQ_CONTROL   (0)
#define GRAIN_DECAY_CONTROL  (2)
#define GRAIN2_FREQ_CONTROL  (3)
#define GRAIN2_DECAY_CONTROL (1)
#define VOLUME_DECAY_CONTROL (5)


// Changing these will also requires rewriting audioOn()

#if defined(__AVR_ATmega8__)
//
// On old ATmega8 boards.
//    Output is on pin 11
//
#define LED_PIN       13
#define LED_PORT      PORTB
#define LED_BIT       5
#define PWM_PIN       11
#define PWM_VALUE     OCR2
#define PWM_INTERRUPT TIMER2_OVF_vect
#elif defined(__AVR_ATmega1280__)
//
// On the Arduino Mega
//    Output is on pin 3
//
#define LED_PIN       13
#define LED_PORT      PORTB
#define LED_BIT       7
#define PWM_PIN       3
#define PWM_VALUE     OCR3C
#define PWM_INTERRUPT TIMER3_OVF_vect
#else
//
// For modern ATmega168 and ATmega328 boards
//    Output is on pin 3
//
#define PWM_PIN       3
#define PWM_VALUE     OCR2B
#define LED_PIN       13
#define LED_PORT      PORTB
#define LED_BIT       5
#define PWM_INTERRUPT TIMER2_OVF_vect
#endif

// Auto-Sequencer
#define MAXIMUM_SEQUENCE_WINDUP 1024
#define MINIMUM_SEQUENCE_WINDUP -1024
#define MAXIMUM_SEQUENCER_MODE 2
#define SEQUENCER_MIN_BPM 40
#define SEQUENCER_MAX_BPM 240
#define SEQUENCER_MODE_MANUAL 0
#define SEQUENCER_MODE_MUSIC_BOX 1
#define SEQUENCER_MODE_CONTINUOUS 2
#define SEQUENCER_MAX_SLOWDOWN_FACTOR 6
int16_t sequencerPosition = 0;
int16_t sequencerTarget = 0;
uint8_t sequencerMode = SEQUENCER_MODE_MUSIC_BOX;
boolean targetReached = true;
uint8_t sequencerBpm = 120;

uint16_t sequencerEventId;
Timer sequencerTimer;

// Volume Decay
#define VOLUME_DECAY_SCALE_FACTOR 50000000
double volumeMultiplierSlope = 0;
double volumeMultiplier = 0;


// Encoders
//
#define TEMPO_ENCODER_PIN_1 4
#define TEMPO_ENCODER_PIN_2 5
AdaEncoder tempoEncoder = AdaEncoder('a', TEMPO_ENCODER_PIN_1, TEMPO_ENCODER_PIN_2);

#define PLAY_ENCODER_PIN_1 6
#define PLAY_ENCODER_PIN_2 7
AdaEncoder playEncoder = AdaEncoder('b', PLAY_ENCODER_PIN_1, PLAY_ENCODER_PIN_2);

// General Button Definitions
#define SINGLE_CLICK                1
#define DOUBLE_CLICK                2
#define LONG_SINGLE_CLICK          -1
#define LONG_DOUBLE_CLICK          -2
#define BUTTON_DEBOUNCE_TIME       20
#define BUTTON_MULTI_CLICK_TIME   250
#define BUTTON_LONG_CLICK_TIME    500


// Melody Encoder Button
//
#define ENCODER_PIN_BUTTON          8
ClickButton button(ENCODER_PIN_BUTTON, LOW, CLICKBTN_PULLUP);

// Phone Button
//
#define PHONE_PIN_BUTTON          10
ClickButton phoneButton(PHONE_PIN_BUTTON, LOW, CLICKBTN_PULLUP);


// Tempo Encoder Button
//
#define TEMPO_ENCODER_PIN_BUTTON         10
#define TEMPO_BUTTON_DEBOUNCE_TIME       20
#define TEMPO_BUTTON_MULTI_CLICK_TIME   250
#define TEMPO_BUTTON_LONG_CLICK_TIME    500
ClickButton tempoButton(TEMPO_ENCODER_PIN_BUTTON, LOW, CLICKBTN_PULLUP);

// Dial "Button"
//
#define DIAL_PIN_BUTTON                  9
#define DIAL_DEBOUNCE_TIME              20
#define DIAL_MULTI_CLICK_TIME_PLAY       1
#define DIAL_MULTI_CLICK_TIME_SELECT    500
#define DIAL_LONG_CLICK_TIME            100
ClickButton dial(DIAL_PIN_BUTTON, LOW, CLICKBTN_PULLUP);
boolean dialIsMelodySelection;

// Smooth logarithmic mapping
//
uint16_t antilogTable[] = {
  64830, 64132, 63441, 62757, 62081, 61413, 60751, 60097, 59449, 58809, 58176, 57549, 56929, 56316, 55709, 55109,
  54515, 53928, 53347, 52773, 52204, 51642, 51085, 50535, 49991, 49452, 48920, 48393, 47871, 47356, 46846, 46341,
  45842, 45348, 44859, 44376, 43898, 43425, 42958, 42495, 42037, 41584, 41136, 40693, 40255, 39821, 39392, 38968,
  38548, 38133, 37722, 37316, 36914, 36516, 36123, 35734, 35349, 34968, 34591, 34219, 33850, 33486, 33125, 32768
};
uint16_t mapPhaseInc(uint16_t input) {
  return (antilogTable[input & 0x3f]) >> (input >> 6);
}

// Stepped chromatic mapping
//
uint16_t midiTable[] = {
  17, 18, 19, 20, 22, 23, 24, 26, 27, 29, 31, 32, 34, 36, 38, 41, 43, 46, 48, 51, 54, 58, 61, 65, 69, 73,
  77, 82, 86, 92, 97, 103, 109, 115, 122, 129, 137, 145, 154, 163, 173, 183, 194, 206, 218, 231,
  244, 259, 274, 291, 308, 326, 346, 366, 388, 411, 435, 461, 489, 518, 549, 581, 616, 652, 691,
  732, 776, 822, 871, 923, 978, 1036, 1097, 1163, 1232, 1305, 1383, 1465, 1552, 1644, 1742,
  1845, 1955, 2071, 2195, 2325, 2463, 2610, 2765, 2930, 3104, 3288, 3484, 3691, 3910, 4143,
  4389, 4650, 4927, 5220, 5530, 5859, 6207, 6577, 6968, 7382, 7821, 8286, 8779, 9301, 9854,
  10440, 11060, 11718, 12415, 13153, 13935, 14764, 15642, 16572, 17557, 18601, 19708, 20879,
  22121, 23436, 24830, 26306
};
uint16_t mapMidi(uint16_t input) {
  return midiTable[wrap(getMidiOffset(input), 0 , 128)];
}

uint16_t getMidiOffset(uint16_t input)
{
  return (1023 - input) >> 3;
}

// Stepped Pentatonic mapping
//
uint16_t pentatonicTable[54] = {
  0, 19, 22, 26, 29, 32, 38, 43, 51, 58, 65, 77, 86, 103, 115, 129, 154, 173, 206, 231, 259, 308, 346,
  411, 461, 518, 616, 691, 822, 923, 1036, 1232, 1383, 1644, 1845, 2071, 2463, 2765, 3288,
  3691, 4143, 4927, 5530, 6577, 7382, 8286, 9854, 11060, 13153, 14764, 16572, 19708, 22121, 26306
};

uint16_t mapPentatonic(uint16_t input) {
  uint8_t value = (1023 - input) / (1024 / 53);
  return (pentatonicTable[value]);
}

// MELODIES!
int16_t currentMelody = 1;
int16_t maxMelody = 2;
int16_t maxMelodySteps[3] = {0, 39, 15};
int16_t lastOffset = 0;

// dirty dirty hack for making melodies start correctly backwards and forwards
double currentStep = -0.5;

// actual melodies start here
// we assume 8ths - so a 4th is actually two steps
int16_t alleMeineEntchenSteps[40] = {
  0, 2, 4, 5, 7, -1, 7, -1, 9, 9, 9, 9, 7, -1, -1, -1, 9, 9, 9, 9, 7, -1, -1, -1, 5, 5, 5, 5, 4, -1, 4, -1, 2, 2, 2, 2, 0, -1, -1, -1
};

int16_t simpleArp[16] = {
  0, 12, 24, 0, 12, 24, 0, 12, 24, 0, 12, 24, 0, 12, 24, 12
};

uint16_t mapMidiMelody(uint16_t input) {
  if (getMelodyOffset(currentStep) != -1)
  {
    lastOffset = midiTable[wrap(getMidiOffset(input) + getMelodyOffset(currentStep), 0, 128)];
  }
  return lastOffset;
}


void audioOn() {
#if defined(__AVR_ATmega8__)
  // ATmega8 has different registers
  TCCR2 = _BV(WGM20) | _BV(COM21) | _BV(CS20);
  TIMSK = _BV(TOIE2);
#elif defined(__AVR_ATmega1280__)
  TCCR3A = _BV(COM3C1) | _BV(WGM30);
  TCCR3B = _BV(CS30);
  TIMSK3 = _BV(TOIE3);
#else
  // Set up PWM to 31.25kHz, phase accurate
  TCCR2A = _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  TIMSK2 = _BV(TOIE2);
#endif
}

void setup() {
  Serial.begin(9600);
  pinMode(PWM_PIN, OUTPUT);
  audioOn();
  pinMode(LED_PIN, OUTPUT);
  button.debounceTime = BUTTON_DEBOUNCE_TIME;
  button.multiclickTime = BUTTON_MULTI_CLICK_TIME;
  button.longClickTime = BUTTON_LONG_CLICK_TIME;

  tempoButton.debounceTime = BUTTON_DEBOUNCE_TIME;
  tempoButton.multiclickTime = BUTTON_MULTI_CLICK_TIME;
  tempoButton.longClickTime = BUTTON_LONG_CLICK_TIME;

  phoneButton.debounceTime = BUTTON_DEBOUNCE_TIME;
  phoneButton.multiclickTime = BUTTON_MULTI_CLICK_TIME;
  phoneButton.longClickTime = BUTTON_LONG_CLICK_TIME;

  dial.debounceTime = DIAL_DEBOUNCE_TIME;
  dial.longClickTime = DIAL_LONG_CLICK_TIME;
  dial.multiclickTime = DIAL_MULTI_CLICK_TIME_SELECT;
  dialIsMelodySelection = true;
}

void loop() {
  // The loop is pretty simple - it just updates the parameters for the oscillators.
  //
  // Avoid using any functions that make extensive use of interrupts, or turn interrupts off.
  // They will cause clicks and poops in the audio.

  // Smooth frequency mapping
  //syncPhaseInc = mapPhaseInc(analogRead(SYNC_CONTROL)) / 4;

  // Stepped mapping to MIDI notes: C, Db, D, Eb, E, F...
  //syncPhaseInc = mapMidi(analogRead(SYNC_CONTROL));

  // Stepped mapping to MIDI plus Melody
  syncPhaseInc = mapMidiMelody(analogRead(SYNC_CONTROL));

  // Stepped pentatonic mapping: D, E, G, A, B
  //syncPhaseInc = mapPentatonic(analogRead(SYNC_CONTROL));

  grainPhaseInc  = mapPhaseInc(analogRead(GRAIN_FREQ_CONTROL)) / 2;
  grainDecay     = analogRead(GRAIN_DECAY_CONTROL) / 8;
  grain2PhaseInc = mapPhaseInc(analogRead(GRAIN2_FREQ_CONTROL)) / 2;
  grain2Decay    = analogRead(GRAIN2_DECAY_CONTROL) / 4;
  volumeMultiplierSlope = pow(analogRead(VOLUME_DECAY_CONTROL), 2) / VOLUME_DECAY_SCALE_FACTOR;
  processEnvelope();

  processTempoEncoder();
  processTempoButton();
  processPlayEncoder();
  processButton();
  processPhoneButton();
  processDial();
  sequencerTimer.update();
}

void processPlayEncoder()
{
  int8_t clicks = 0;
  clicks = playEncoder.query();

  if (clicks == 0) return;

  doModeStep(-clicks);
}

void processTempoEncoder()
{
  int8_t clicks = 0;
  clicks = tempoEncoder.query();
  sequencerBpm = max(min(sequencerBpm + clicks, SEQUENCER_MAX_BPM), SEQUENCER_MIN_BPM);
}

void doModeStep(int steps)
{
  if (sequencerMode == SEQUENCER_MODE_MUSIC_BOX)
  {
    moveTargetPosition(steps);
  } else
  {
    stepMelodyNote(steps);
  }  
}


void processButton()
{
  button.Update();
  if (button.clicks != 0)
  {
    if (button.clicks == SINGLE_CLICK)
    {
      stepMelodySelection(1);
    } else if (button.clicks == DOUBLE_CLICK)
    {
      stepMelodySelection(-1);
    } else if (button.clicks = LONG_SINGLE_CLICK)
    {
      toggleAutoSequencer();
    }
  }
}

void processTempoButton()
{
  tempoButton.Update();
  if (tempoButton.clicks != 0)
  {
    if (tempoButton.clicks == SINGLE_CLICK)
    {
      toggleDialMode();
    }
  }
}

void processPhoneButton()
{
  phoneButton.Update();
  if (phoneButton.clicks != 0)
  {
    Serial.print("Phone button click registered: ");
    Serial.println(phoneButton.clicks);
  }
}

void toggleDialMode()
{
  dialIsMelodySelection = !dialIsMelodySelection;

  if (dialIsMelodySelection){
    dial.multiclickTime = DIAL_MULTI_CLICK_TIME_SELECT;
  } else
  {
    dial.multiclickTime = DIAL_MULTI_CLICK_TIME_PLAY;
  }
}

void processDial()
{
  dial.Update();

  if (dial.clicks != 0)
  {
    Serial.print("Dial click registered: ");
    Serial.println(dial.clicks);
    if (dialIsMelodySelection)
    {
      int clicks = -dial.clicks;
      if (clicks != 0)
      {
        if (clicks == 10) clicks = 0;
        currentMelody = min(clicks, maxMelody);
        Serial.print("Dial click: ");
        Serial.println(clicks);
        Serial.print("Melody: ");
        Serial.println(currentMelody);
      }
    } else if (dial.clicks > 0) {
      doModeStep(1);
    }
  }
}

void processEnvelope()
{
  volumeMultiplier = max(volumeMultiplier - volumeMultiplierSlope, 0);
}

void toggleAutoSequencer() {
  sequencerTimer.stop(sequencerEventId);

  sequencerMode = wrap(sequencerMode + 1, 0, MAXIMUM_SEQUENCER_MODE);

  if (sequencerMode == SEQUENCER_MODE_MUSIC_BOX)
  {
    Serial.println("Music Box mode");
    targetReached = true;
  } else if (sequencerMode == SEQUENCER_MODE_MANUAL)
  {
    Serial.println("Manual mode");
  } else if (sequencerMode == SEQUENCER_MODE_CONTINUOUS)
  {
    Serial.println("Continuous Run mode");
    sequencerEventId = sequencerTimer.every(getSequencerInterval(), singleMelodyNoteStep);
  }
}

int16_t getMelodyOffset(int16_t melodyPosition)
{
  switch (currentMelody)
  {
    case 1 :
      return alleMeineEntchenSteps[melodyPosition];
      break;
    case 2 :
      return simpleArp[melodyPosition];
      break;
    default:
      return 0;
  }
}

int16_t wrap(int16_t kX, int16_t kLowerBound, int16_t kUpperBound)
{
  int16_t range_size = kUpperBound - kLowerBound + 1;

  if (kX < kLowerBound)
    kX += range_size * ((kLowerBound - kX) / range_size + 1);

  return kLowerBound + (kX - kLowerBound) % range_size;
}

void stepMelodySelection(int16_t stepSize)
{
  currentMelody = wrap(currentMelody + stepSize, 0, maxMelody);
  Serial.print("Melody: ");
  Serial.println(currentMelody);
}

void singleMelodyNoteStep()
{
  stepMelodyNote(1);
  Serial.println(volumeMultiplierSlope);
}

void sequencerStep()
{
  if (sequencerTarget == sequencerPosition)
  {
    Serial.println("Reached target - disabling sequencer");
    sequencerTimer.stop(sequencerEventId);
    targetReached = true;
  } else
  {
    if (sequencerTarget > sequencerPosition)
    {
      stepMelodyNote(1);
      sequencerPosition++;
    }
    else if (sequencerTarget < sequencerPosition)
    {
      stepMelodyNote(-1);
      sequencerPosition--;
    }

    // Run twice as slow if we are close to the end
    int16_t diff = abs(sequencerTarget - sequencerPosition);

    int16_t nextStepDelay = getSequencerInterval();
    nextStepDelay *= max(SEQUENCER_MAX_SLOWDOWN_FACTOR - diff, 1);
    scheduleSequencerStep(nextStepDelay);
  }
}

void scheduleSequencerStep(uint16_t nextStepDelay)
{
  sequencerTimer.stop(sequencerEventId);
  sequencerEventId = sequencerTimer.every(nextStepDelay, sequencerStep);
}

void stepMelodyNote(int16_t stepSize)
{
  currentStep = wrap(currentStep + stepSize, 0, getCurrentMelodyMaxStep());

  if (getMelodyOffset(currentStep) != -1)
  {
    volumeMultiplier = 1;
  }
  Serial.print("Step: ");
  Serial.println(currentStep);
}

int16_t getCurrentMelodyMaxStep()
{
  return maxMelodySteps[currentMelody];
}

void moveTargetPosition(int16_t stepSize)
{
  // if we were stopped before, start the timer now, since we moved away from the target position
  if (targetReached == true)
  {
    scheduleSequencerStep(getSequencerInterval());
    targetReached = false;
  }
  sequencerTarget = max(min(sequencerTarget + stepSize, MAXIMUM_SEQUENCE_WINDUP), MINIMUM_SEQUENCE_WINDUP);
}

uint8_t getSequencerInterval()
{
  // we're calculating with 8ths, so actually calculate beats per half-minute
  return 30000 / sequencerBpm;
}

SIGNAL(PWM_INTERRUPT)
{
  uint8_t value;
  uint16_t output;

  syncPhaseAcc += syncPhaseInc;
  if (syncPhaseAcc < syncPhaseInc) {
    // Time to start the next grain
    grainPhaseAcc = 0;
    grainAmp = 0x7fff;
    grain2PhaseAcc = 0;
    grain2Amp = 0x7fff;
    LED_PORT ^= 1 << LED_BIT; // Faster than using digitalWrite
  }

  // Increment the phase of the grain oscillators
  grainPhaseAcc += grainPhaseInc;
  grain2PhaseAcc += grain2PhaseInc;

  // Convert phase into a triangle wave
  value = (grainPhaseAcc >> 7) & 0xff;
  if (grainPhaseAcc & 0x8000) value = ~value;
  // Multiply by current grain amplitude to get sample
  output = value * (grainAmp >> 8);

  // Repeat for second grain
  value = (grain2PhaseAcc >> 7) & 0xff;
  if (grain2PhaseAcc & 0x8000) value = ~value;
  output += value * (grain2Amp >> 8);

  // Make the grain amplitudes decay by a factor every sample (exponential decay)
  grainAmp -= (grainAmp >> 8) * grainDecay;
  grain2Amp -= (grain2Amp >> 8) * grain2Decay;

  // Scale output to the available range, clipping if necessary
  output >>= 9;
  if (output > 255) output = 255;

  output *= volumeMultiplier;

  // Output to PWM (this is faster than using analogWrite)
  PWM_VALUE = output;
}
