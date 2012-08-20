#include <octave/oct.h>
#include <octave/octave.h>
#include <octave/parse.h>
#include <octave/toplev.h> /* do_octave_atexit */
#include <iostream>

#include <MT/array.h>
#include <MT/util.h>

void func(arr& y, const arr& x){
  CHECK(x.nd==2,"");
  Matrix _x(x.d0, x.d1);
  for(uint i=0;i<x.d0;i++)
    for(uint j=0;j<x.d1;j++)
      _x(i,j) = x(i,j);

  octave_value_list functionArguments;
  functionArguments (0) = _x;

  const octave_value_list result = feval ("func", functionArguments, 1);

  Matrix _y = result(0).matrix_value();

  y.resize(_y.dim1(),_y.dim2());

  for(uint i=0;i<y.d0;i++)
    for(uint j=0;j<y.d1;j++)
      y(i,j) = _y(i,j);
}

int main (const int argc, char ** argv){
   const char * argvv [] = {"" /* name of program, not relevant */,  
"--silent"};

   octave_main (2, (char **) argvv, true /* embedded */);

   arr x,y;
   x <<"[1 2 3; 3 4 5]";
   func(y,x);
   cout <<y <<'\n' <<~x*x <<endl;
  
   return 0;
}

