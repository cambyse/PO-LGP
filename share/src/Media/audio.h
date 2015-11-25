#pragma once

#include <Core/array.h>


struct SineSound{
  float sampleRate;
  floatA notes; //four entries per note: (sin-buffer-step-size, amplitude, time, decay)
  floatA SIN;

  SineSound(float _sampleRate=16000);

  void addNote(float freq, float a=.1, float decay=0.0007);
  void changeFreq(uint i,float freq);
  void reset();
  void clean();
  float get();
};

//===========================================================================

struct Audio{
  void *stream;
  Audio(SineSound& S);
  ~Audio();
};
