#include <stdio.h>
#include "portaudio.h"

int main(void)
{
    PaStreamParameters outputParameters;
    PaStream *stream;
    PaError err;
//    paTestData data;
    int i;
    printf("PortAudio Test: output sine wave.\n");


    err = Pa_Initialize();
//    if( err != paNoError ) goto error;

    outputParameters.device = Pa_GetDefaultOutputDevice(); /* default output device */
    if (outputParameters.device == paNoDevice) {
      fprintf(stderr,"Error: No default output device.\n");
//      goto error;
    }
    outputParameters.channelCount = 2;                     /* stereo output */
    outputParameters.sampleFormat = paFloat32;             /* 32 bit floating point output */
    outputParameters.suggestedLatency = Pa_GetDeviceInfo( outputParameters.device )->defaultLowOutputLatency;
    outputParameters.hostApiSpecificStreamInfo = NULL;


    printf("Test finished.\n");
    return err;

}

