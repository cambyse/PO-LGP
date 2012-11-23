#include "octave.h"

#ifdef MT_OCTAVE

//#include <octave/toplev.h> /* do_octave_atexit */
#include <octave/octave.h>

bool octaveInitialized=false;

void octaveCheckInitialized(){
  if(!octaveInitialized){
    const char * argvv [] = {"", "--silent"};
    octave_main (2, (char **) argvv, true);
    octaveInitialized=true;
  }
}

#else

#include "util.h"
void octaveCheckInitialized(){ NICO }

#endif
