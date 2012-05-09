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
