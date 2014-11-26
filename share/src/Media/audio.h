#pragma once

#include <Core/array.h>

#define SAMPLE_RATE 16000

struct SineSound{
  float sampleRate;
  floatA notes; //four entries per note: (sin-buffer-step-size, amplitude, time, decay)
  floatA SIN;

  SineSound(float _sampleRate=SAMPLE_RATE);

  void addNote(float freq, float a=.1, float decay=0.0007);
  void changeFreq(uint i,float freq);
  void reset();
  void clean();
  float get();
};

//===========================================================================

struct Audio{
  struct sAudio *s;

  Audio();
  ~Audio();

  void open(SineSound& S);
  void close();
};
