
// Roy Hotrabhvanon SDP Group 7 2015
#include "Arduino.h"

#ifndef Rotary_h
#define Rotary_h



// Enable this to emit codes twice per step.
// #define HALF_STEP

// Enable weak pullups
#define ENABLE_PULLUPS

// Values returned by 'process'
// No complete step yet.
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Counter-clockwise step.
#define DIR_CCW 0x20

class Rotary
{
  public:
    Rotary(char, char);
    unsigned char process();
  private:
    unsigned char state;
    unsigned char pin1;
    unsigned char pin2;
};

#endif
 