#include <Optim/optimization.h>
#include <Optim/benchmarks.h>

#include <iomanip>

#include "lecture.cpp"


//==============================================================================
//
// test standard constrained optimizers
//

void testConstraint2(ConstrainedProblem& p, arr& x_start=NoArr){
  //-- initial x
  arr x(p.dim_x());
  if(&x_start) x=x_start;
  else x.setZero();
  cout <<"x0=" <<x <<endl;
  rnd.seed(0);

  optConstrained(x, NoArr, p);

  if(&x_start) x_start = x;
}

//==============================================================================
//
// test the phase one optimization
//

void testPhaseOne(ConstrainedProblem& f){
  PhaseOneProblem metaF(f);

  arr x;
  x = ARRAY(1., 1., 10.);

  testConstraint(metaF, x, 1);
  //one iteration of phase one should be enough
  //properly done: check in each step if constraints are fulfilled and exit phase one then
  //no need to really minimize

  x=x.sub(0,-2);
  testConstraint(f, x);
}

//==============================================================================

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  ChoiceConstraintFunction F;
//  SimpleConstraintFunction F;
//  testConstraint(F);
  testConstraint2(F);

  return 0;
}
