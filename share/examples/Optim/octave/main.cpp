//#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <Optim/newton.h>
#include <Octave/octave.h>
#include <octave/oct.h>
#include <octave/octave.h>
#include <octave/parse.h>
//#include <Core/array.h>

//==============================================================================

int main (const int argc, char ** argv){
  mlr::initCmdLine(argc,argv);

  //--startup octave
  const char * argvv [] = {"", "--silent"};
  octave_main (2, (char **) argvv, true);

  //-- set the scalar function that is DEFUN_DLD-imported into octave
  OCTAVE().scalarFunction = ChoiceFunction();
  OCTAVE().verbose = 1;

  //-- just check calling the function
   arr x =  .3*randn(mlr::getParameter<uint>("dim",4));
   cout <<"x=" <<x <<endl;

   arr x2=x;

   octave_value_list args;
   args(0) = conv_arr2octave(x);
   int i;
//   args = eval_string("A=[1,2,3]", false, i); //, args, 2);
//   args = eval_string("A", false, i); //, args, 2);
   args = feval("mlr_ScalarFunction", args, 3);

   arr X = conv_octave2arr(args(0).matrix_value());
   arr g = conv_octave2arr(args(1).matrix_value());
   cout <<"X=" <<X <<"g=" <<g <<endl;

   //-- call fminunc on the function
   args.clear();
   args(0) = "mlr_ScalarFunction";
   args(1) = conv_arr2octave(x);
   args(2) = eval_string("optimset('GradObj', 'on');", false, i);
   feval("fminunc", args);

   optNewton(x2, OCTAVE().scalarFunction, OPT(verbose=2));

   return 0;
}

//===========================================================================
