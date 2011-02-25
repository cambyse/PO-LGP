#ifdef MT_TONY

#include <mdp/mdp.h>
#include "array.h"
#include "mdp.h"

void MatrixToArray(arr& X,Matrix M,uint d0,uint d1){
  CHECK(d0==(uint)M->num_rows,"given dimensionality not consistent");
  X.resize(d0,d1);
  X.setZero();
  uint i, j;
  for(i=0;i<(uint)M->num_rows;i++ ) {
    for(j=M->row_start[i]; j<(uint)(M->row_start[i]+M->row_length[i]); j++)
      X(i,M->col[j])= M->mat_val[j];
  }
}

void mdp::readMDP_POMDP(MDP& mdp, const char *filename){
  /*
    This routine returns 1 if the file is successfully parsed and 0 if not.
  */
  cout <<"parsing pomdp file `" <<filename <<"' using Cassandra's mdp library" <<endl;

  ::readMDP( (char*)filename );

  uint a,nx=gNumStates, ny=gNumObservations, na=gNumActions;
  //cout <<"nx=" <<nx <<"  ny=" <<ny <<"  na=" <<na <<endl;

  //cout <<"Paxy =\n"; for(a=0;a<na;a++){ cout <<"[a="<<a<<"]"; displayMatrix(R[a]); }
  //cout <<"Paxx =\n"; for(a=0;a<na;a++){ cout <<"[a="<<a<<"]"; displayMatrix(P[a]); }
  //cout <<"Q(a,x) =\n"; displayMatrix(Q);

  arr Paxy(na,nx,ny);
  for(a=0;a<na;a++) MatrixToArray(Paxy[a](),R[a],nx,ny);
  //cout <<"Paxy =\n" <<Paxy <<endl;

  mdp.Pyxa.resize(ny,nx,na); //a permutation of Paxy
  mdp.Pyxa=1.;
  tensorMultiply(mdp.Pyxa,Paxy,TUP(2,1,0));
  //cout <<"Pyax =\n" <<mdp.Pyxa <<endl;
  ::checkNormalization(mdp.Pyxa);

  arr Paxx(na,nx,nx);
  for(a=0;a<na;a++) MatrixToArray(Paxx[a](),P[a],nx,nx);
  //cout <<"Paxx =\n" <<Paxx <<endl;
  mdp.Pxax.resize(nx,na,nx);
  mdp.Pxax=1.;
  tensorMultiply(mdp.Pxax,Paxx,TUP(1,2,0));
  //cout <<"mdp.Pxax =\n" <<mdp.Pxax <<endl;
  ::checkNormalization(mdp.Pxax,1e-4);
  tensorCondNormalize(mdp.Pxax,1);

  mdp.Rax.resize(na,nx);
  MatrixToArray(mdp.Rax,Q,na,nx);
  //cout <<"mdp.Rax =\n" <<mdp.Rax <<endl;

  mdp.Px.setCarray(gInitialBelief,nx);
  //cout <<"mdp.Px =\n" <<mdp.Px <<endl;
  ::checkNormalization(mdp.Px,1e-4);
  tensorCondNormalize(mdp.Px,1);

  mdp.gamma = gDiscount;
}

#else

#include "mdp.h"
void mdp::readMDP_POMDP(MDP& mdp, const char *filename){
  HALT("reading Tony's POMDP file only works for Linux");
}

#endif

