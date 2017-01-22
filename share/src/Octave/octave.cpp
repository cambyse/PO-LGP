#include "octave.h"

#include <octave/oct.h>
//#include <octave/octave.h>

uint fs_eval=0;

Singleton<OctaveSpace> OCTAVE;

OctaveSpace::OctaveSpace()
  : verbose(0){
}

//===========================================================================

Matrix conv_arr2octave(const arr& x){
  if(x.nd==2){
    //octave stores COLUMN-MAJOR!!
    Matrix oct_x(x.d1, x.d0);
    memmove((double*)oct_x.data(), x.p, x.sizeT*x.N);
    return oct_x.transpose();
  }
  if(x.nd==1){
    Matrix oct_x(x.d0,1);
    memmove((double*)oct_x.data(), x.p, x.sizeT*x.N);
    return oct_x;
  }
  HALT("");
  return Matrix(1,1);
}

arr conv_octave2arr(const Matrix& x){
  arr y(x.dim1(),x.dim2());
  memmove(y.p, (double*)x.transpose().data(), y.sizeT*y.N);
  return y;
}

//===========================================================================

DEFUN_DLD (helloworld, args, nargout,
           "Hello World Help String")
{
  int nargin = args.length ();

  octave_stdout << "Hello MARC World has "
                << nargin << " input arguments and "
                << nargout << " output arguments.\n";

  return octave_value_list ();
}

DEFUN_DLD (mlr_ScalarFunction, args, nargout,
           "ScalarFunction: x -> [f,g,H]")
{
  if(args.length()!=1){
    octave_stdout << "ScalarFunction requires ONE input argument\n";
    return octave_value_list ();
  }

  Matrix X(args(0).matrix_value());
  arr x(X.data(), X.numel());
//  octave_stdout <<"input vec = " <<x <<endl;

  if(!OCTAVE().scalarFunction){
    octave_stdout << "the ScalarFunction in the singleton OctaveSpace has not been defined" <<endl;
    return octave_value(X);
  }

  fs_eval++;

  if(nargout<=1){
    double f = OCTAVE().scalarFunction(NoArr, NoArr, x);
    if(OCTAVE().verbose>0) cout <<"OCT_fs_1: #" <<fs_eval <<" \tf=" <<f <<" \tx=" <<(x.N<5?STRING(x):STRING('[' <<x.N <<']')) <<endl;
    return octave_value(f);
  }

  if(nargout==2){
    arr g;
    double f = OCTAVE().scalarFunction(g, NoArr, x);
    octave_value_list out;
    out(0) = f;
    out(1) = conv_arr2octave(g);
    if(OCTAVE().verbose>0) cout <<"OCT_fs_2: #" <<fs_eval <<" \tf=" <<f <<" \tx=" <<(x.N<5?STRING(x):STRING('[' <<x.N <<']')) <<endl;
    return out;
  }

  if(nargout==3){
    arr g, H;
    double f = OCTAVE().scalarFunction(g, H, x);
    octave_value_list out;
    out(0) = f;
    out(1) = conv_arr2octave(g);
    out(2) = conv_arr2octave(H);
    if(OCTAVE().verbose>0) cout <<"OCT_fs_3: #" <<fs_eval <<" \tf=" <<f <<" \tx=" <<(x.N<5?STRING(x):STRING('[' <<x.N <<']')) <<endl;
    return out;
  }

  octave_stdout << "wrong nargout=" <<nargout <<endl;
  return octave_value(X);
}

//===========================================================================

