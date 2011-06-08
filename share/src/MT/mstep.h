#ifndef MT_mstep_h
#define MT_mstep_h

#include "array.h"

//enum MstepType{ MstepNoisyMax, MstepExact, MstepNone, MstepCopyExpectations };

void standardMstep(arr& param,const arr& post,uint left,double noise=0);

void noisyMaxMstep_old(arr& param,const arr& post,uint left);
void noisyMaxMstep(arr& param,const arr& post,uint left,double rate,double noise=0);
void noisyMaxMstep2(arr& param,const arr& likelihood,uint left,double noise=0);
void mstep_11rule(arr& param,const arr& post,uint left,double rate,double noise=0);

#ifdef  MT_IMPLEMENTATION
#  include "mstep.cpp"
#endif

#endif
