#include <portaudio.h>
#include "audio.h"

SineSound::SineSound(float sampleRate):t(0), dt(1.f/sampleRate){}

void SineSound::addNote(float freq, float a){
  floatA note = { float(MT_2PI*freq), a, t };
  notes.append( note );
  notes.reshape(notes.N/3, 3);
}

void SineSound::reset(){ notes.clear(); }

void SineSound::clean(){
  for(uint i=notes.d0;i--;){
    if(notes(i,1)<1e-4) notes.delRows(i);
  }
}

float SineSound::get(){
  double x=0.;
  for(uint i=0;i<notes.d0; i++){
    float &a=notes(i, 1);
    x += a * sin(notes(i,0) * (t-notes(i,2)));
    if(a>0.05) a *= .995;
    else a *= .9993;
  }
  t += dt;
  return x;
}

//===========================================================================

struct sAudio{
  PaStream *stream;
};

void err(PaError e){
  if(!e) return;
  Pa_Terminate();
  HALT("PortAudio error" <<e <<": " <<Pa_GetErrorText( e ) );
}

static int PortAudioCallback( const void *inputBuffer, void *outputBuffer,
                            unsigned long framesPerBuffer,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags,
                            void *userData ){
  SineSound &s = *((SineSound*)userData);
  float *out = (float*)outputBuffer;
  unsigned long i;

  (void) timeInfo; /* Prevent unused variable warnings. */
  (void) statusFlags;
  (void) inputBuffer;

  s.clean();
  for( i=0; i<framesPerBuffer; i++ ) *out++ = s.get();

  return paContinue;
}

Audio::Audio():s(NULL){
  s = new sAudio;
}

Audio::~Audio(){
  delete s;
  s=NULL;
}

void Audio::open(SineSound& S){
  err( Pa_Initialize() );

  PaStreamParameters outputParameters;
  outputParameters.device = Pa_GetDefaultOutputDevice(); /* default output device */
  if (outputParameters.device == paNoDevice) err(paNoDevice);
  outputParameters.channelCount = 1; /* stereo output */
  outputParameters.sampleFormat = paFloat32; /* 32 bit floating point output */
  outputParameters.suggestedLatency = Pa_GetDeviceInfo( outputParameters.device )->defaultLowOutputLatency;
  outputParameters.hostApiSpecificStreamInfo = NULL;

  err( Pa_OpenStream(
         &s->stream,
         NULL, /* no input */
         &outputParameters,
         SAMPLE_RATE,
         64, //frames per buffer
         0,
         PortAudioCallback,
         &S ) );

  err( Pa_StartStream( s->stream ) );
}

void Audio::close(){
  err( Pa_StopStream( s->stream ) );
  err( Pa_CloseStream( s->stream ) );
  err( Pa_Terminate() );
}


