#include "octave.h"

#ifdef MT_OCTAVE

#include <octave/toplev.h> /* do_octave_atexit */

bool octaveInitialized=false;

void octaveCheckInitialized(){
  if(!octaveInitialized){
    const char * argvv [] = {"", "--silent"};
    octave_main (2, (char **) argvv, true);
    octaveInitialized=true;
  }
}

#else

void octaveCheckInitialized(){ NICO }

#endif
