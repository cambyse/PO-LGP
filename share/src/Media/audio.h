#pragma once

#include <Core/array.h>

#define SAMPLE_RATE 16000

struct SineSound{
  float t, dt;
  floatA notes;

  SineSound(float sampleRate=SAMPLE_RATE);

  void addNote(float freq, float a=.1);
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
