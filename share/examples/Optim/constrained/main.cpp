#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include "problems.h"

//lecture.cpp:
void testConstraint(ConstrainedProblem& p, arr& x_start=NoArr, uint iters=20);

//==============================================================================
//
// test standard constrained optimizers
//

void testConstraint2(ConstrainedProblem& p, arr& x_start=NoArr){
  //-- initial x
  arr x = zeros(p.dim_x());
  if(&x_start) x=x_start;
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

void testCoveringSphere(){
  uint n=100, s=10;
  arr x(n,3); rndGauss(x);
  CoveringSpheresProblem F(x,s);

  //-- initial x
  arr cr = F.initialization(x);
  rndGauss(cr, .01);

  cout <<"point = " <<x <<endl;
  cout <<"cr_init=" <<cr <<endl;
  checkAllGradients(F, cr, 1e-4);
  optConstrained(cr, NoArr, F);
  cout <<"cr_opt=" <<cr <<endl;
}

//==============================================================================

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  ChoiceConstraintFunction F;
//  RandomLPFunction F;
//  SimpleConstraintFunction F;
//  testConstraint(F);
  testConstraint2(F);

//  testCoveringSphere();

  return 0;
}
