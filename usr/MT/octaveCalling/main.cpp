#include <MT/octave.h>

arr func(const arr& A, const arr& B){
  octave_value_list args;
  args(0) = octave(A);
  args(1) = octave(B);
  args = feval ("func", args, 2);
  return octave(args(0).matrix_value());
}


int main (const int argc, char ** argv){

   arr A(2,4), B(4,3);
   rndInteger(A,0,3);
   rndInteger(B,0,3);
   cout <<"A=" <<A <<"\nB=" <<B <<endl;
   cout <<func(A,B) <<'\n' <<A*B <<endl;
  
   return 0;
}

