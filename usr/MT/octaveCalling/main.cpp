#include <MT/octave.h>

void func(arr& y, const arr& x){
  octave_value_list args;
  args(0) = octave(x);
  args = feval ("func", args, 1);
  y = octave(args(0).matrix_value());
}


int main (const int argc, char ** argv){

   arr x,y;
   x <<"[1 2 3; 3 4 5]";
   func(y,x);
   cout <<y <<'\n' <<~x*x <<endl;
  
   return 0;
}

