#include <Optim/optimization.h>
#include <Optim/benchmarks.h>

void testForManyLPs(){
  ConstrainedMethodType constrainedMethod = (ConstrainedMethodType)MT::getParameter<int>("opt/constrainedMethod",augmentedLag);
  ofstream fil(STRING("z.all."<<MethodName[constrainedMethod]));

  rnd.seed(2);
  uintA LPsizes={ 20, 50, 100, 200, 500 };
  uint K=10;

  for(uint s:LPsizes){
    for(uint k=0;k<K;k++){
      cout <<"testing size " <<s <<" trial " <<k <<endl;
      //-- initial x
      arr x = zeros(s);

      ChoiceConstraintFunction F;
      F.n = s;

      uint evals = optConstrained(x, NoArr, F, OPT(verbose=0));
      fil <<"size=" <<s <<" evals=" <<evals <<' ';
      evaluateConstrainedProblem(x, F, fil);
    }
  }
}

//==============================================================================

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  testForManyLPs();

  return 0;
}
