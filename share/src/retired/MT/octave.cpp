/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */
#include "octave.h"

#ifdef MLR_OCTAVE

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

#include <Core/util.h>
void octaveCheckInitialized(){ NICO }

#endif
