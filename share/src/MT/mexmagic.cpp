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


#ifdef MT_MEX

#include "mexmagic.h"

MT::String mout;
bool mexVerbose=false;

int _nlhs, _nrhs;
mxArray **_plhs;
const mxArray **_prhs;

void mexFlushHandler(MT::String& string){
  mexPrintf(string.p); string.clear();
}

void initMex(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]){
  _nlhs=nlhs;
  _plhs=plhs;
  _nrhs=nrhs;
  _prhs=prhs;
  mout.flushHandler=mexFlushHandler;
  char *cmd=mxArrayToString(prhs[0]);
  if(!strcmp(cmd, "verbose")){ mexVerbose ^= 1; }
  if(mexVerbose){
    mout <<"MEX COMMAND = " <<cmd <<endl;
    mout <<"MEX INPUTS  = " <<endl;
    for(int i=1; i<_nrhs; i++){
      if(mxGetClassID(_prhs[i])!=mxDOUBLE_CLASS)
        mout <<i <<": <not a double array>" <<endl;
      else
        mout <<i <<":\n" <<RHS(i) <<endl;
    }
    mout <<"MEX #OUTPUTS = " <<nlhs <<endl;
  }
}

void byeMex(){
  if(mexVerbose){
    mout <<"MEX OUTPUTS  = " <<endl;
    for(int i=0; i<_nlhs; i++) mout <<i <<":\n" <<LHSnonew(i) <<endl;
  }
}

#else
#endif
