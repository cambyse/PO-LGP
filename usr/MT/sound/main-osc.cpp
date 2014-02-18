/*
 * The text above constitutes the entire PortAudio license; however,
 * the PortAudio community also makes the following non-binding requests:
 *
 * Any person wishing to distribute modifications to the Software is
 * requested to send the modifications to the original developer so that
 * they can be incorporated into the canonical version. It is also
 * requested that these non-binding requests be included along with the
 * license above.
 */
#include <stdio.h>
#include <math.h>
#include <portaudio.h>
#include <Core/util.h>
#include <Core/array.h>

#define SAMPLE_RATE (8000)

arr rk4(const arr& x0, const arr& A, double dt){
  arr k1,k2,k3,k4;
  k1 = A * x0;
  k2 = A * (x0 + 0.5*dt*k1);
  k3 = A * (x0 + 0.5*dt*k2);
  k4 = A * (x0 +     dt*k3);
  return x0 + (dt/6.)*(k1 + 2.*k2 + 2.*k3 + k4);
}


struct SineSound{
  arr x,A;

  SineSound(float freq=880){ add(freq); }
  ~SineSound(){}

  void add(double freq, double xi=10, double amp=.1){
    double omega = MT_2PI*freq;
    double Kp = omega*omega + xi*xi;
    double Kd = 2.*xi;
    x.append( ARR(amp,0.) );
    arr Abig(A.d0+2,A.d0+2);
    Abig.setZero();
    if(A.N) Abig.setMatrixBlock(A,0,0);
    Abig.setMatrixBlock( ARR(0.,1.,-Kp,-Kd).reshape(2,2) ,A.d0, A.d0);
    A=Abig;
  }

  void reset(){ x=ARR(1,0); }
  float get(){
    x = rk4(x,A,1./8000);
    double sum=0.;
    for(uint i=0;i<x.N;i+=2) sum += x(i);
    return sum;
  }

};

/* This routine will be called by the PortAudio engine when audio is needed.
** It may called at interrupt level on some machines so don't do anything
** that could mess up the system like calling malloc() or free().
*/
static int patestCallback( const void *inputBuffer, void *outputBuffer,
                           unsigned long framesPerBuffer,
                           const PaStreamCallbackTimeInfo* timeInfo,
                           PaStreamCallbackFlags statusFlags,
                           void *userData )
{
  SineSound &s = *((SineSound*)userData);
  float *out = (float*)outputBuffer;
  unsigned long i;

  (void) timeInfo; /* Prevent unused variable warnings. */
  (void) statusFlags;
  (void) inputBuffer;

  for( i=0; i<framesPerBuffer; i++ ){
    float x=s.get();
    *out++ = x;
    *out++ = x;
  }

  return paContinue; //s.done()?paComplete:paContinue;
}

/*
 * This routine is called by portaudio when playback is done.
 */
static void StreamFinished( void* userData )
{
  printf( "Stream Completed: \n" );
}

/*******************************************************************/
int main(void){
  PaStreamParameters outputParameters;
  PaStream *stream;
  PaError err;
  //paTestData data;

  SineSound S;


  err = Pa_Initialize();
  if( err != paNoError ) goto error;

  outputParameters.device = Pa_GetDefaultOutputDevice(); /* default output device */
  if (outputParameters.device == paNoDevice) {
    fprintf(stderr,"Error: No default output device.\n");
    goto error;
  }
  outputParameters.channelCount = 2; /* stereo output */
  outputParameters.sampleFormat = paFloat32; /* 32 bit floating point output */
  outputParameters.suggestedLatency = Pa_GetDeviceInfo( outputParameters.device )->defaultLowOutputLatency;
  outputParameters.hostApiSpecificStreamInfo = NULL;

  err = Pa_OpenStream(
          &stream,
          NULL, /* no input */
          &outputParameters,
          SAMPLE_RATE,
          64, /* frames per buffer */
          paClipOff, /* we won't output out of range samples so don't bother clipping them */
          patestCallback,
          &S );
  if( err != paNoError ) goto error;

  err = Pa_SetStreamFinishedCallback( stream, &StreamFinished );
  if( err != paNoError ) goto error;

  err = Pa_StartStream( stream );
  if( err != paNoError ) goto error;

  MT::wait(.1);

  S.add(440);
//  S.reset();
//  err = Pa_StartStream( stream );
  MT::wait(10.);

  err = Pa_StopStream( stream );
  if( err != paNoError ) goto error;

  err = Pa_CloseStream( stream );
  if( err != paNoError ) goto error;

  Pa_Terminate();
  printf("Test finished.\n");

  return err;
error:
  Pa_Terminate();
  fprintf( stderr, "An error occured while using the portaudio stream\n" );
  fprintf( stderr, "Error number: %d\n", err );
  fprintf( stderr, "Error message: %s\n", Pa_GetErrorText( err ) );
  return err;
}
