#include <stdlib.h>
#include <MT/roboticsCourse.h>

//===========================================================================
//
// abtraction of a general trajectory optimization problem
//

struct ProblemAbstraction{
  virtual void getPhi(arr& Phi, arr& PhiJ, const arr& q, uint t) = 0;
  virtual void getPsi(arr& Psi, arr& PsiA, arr& PsiB, const arr& qt_1, const arr& qt, uint t) = 0;
  virtual void getW  (arr& W, const arr& qt_1, const arr& qt, uint t) = 0;

  double costAndGradient(arr* grad,const arr& q);
};

double ProblemAbstraction::costAndGradient(arr* grad,const arr& q){ //(pointers are optional returns)
  uint T = q.d0-1;
  uint t;
  double taskCosts,controlCosts,cost=0.;
  if(grad){
    grad->resize(T+1,q.d1);
    grad->setZero();
  }
  arr Phi,Psi,J,A,B,W;
  ofstream fil("cost");
  for(t=1;t<=T;t++){
    getPhi(Phi,J,q[t],t);
    getPsi(Psi,A,B,q[t-1],q[t],t);
    getW(W,q[t-1],q[t],t);

    taskCosts    = ( ~Phi*Phi )(0);  //the ( ... )(0) converts the 1x1-arr to a double... 
    controlCosts = ( ~Psi*W*Psi )(0);
    fil <<t <<' ' <<taskCosts <<' ' <<controlCosts <<endl;
    cost += taskCosts + controlCosts;

    if(grad){
      (*grad)[t]()   += 2.*~Phi*J;
      (*grad)[t]()   += 2.*~Psi*W*B;
      (*grad)[t-1]() += 2.*~Psi*W*A;
    }
  }
  return cost;
}


//===========================================================================
//
// a specific trajectory optimization problem implemented using our simulator
//

struct TrajectoryOptimizationProblem:ProblemAbstraction{
  Simulator S;
  uint T;
  arr q0;

  TrajectoryOptimizationProblem();
  void getPhi(arr& Phi, arr& PhiJ, const arr& q, uint t);
  void getPsi(arr& Psi, arr& PsiA, arr& PsiB, const arr& qt_1, const arr& qt, uint t);
  void getW(arr& W, const arr& qt_1, const arr& qt, uint t);
};

TrajectoryOptimizationProblem::TrajectoryOptimizationProblem():S("man.ors"){
  T=10;
  S.getJointAngles(q0);
}

void TrajectoryOptimizationProblem::getPhi(arr& Phi, arr& PhiJ, const arr& q, uint t){
  //RECALL EXERCISE 3: where we also attached multiple tasks to one big task vector...

  arr y_target,y,J,reference;
  Phi.clear();
  J.clear();
  S.setJointAngles(q,false);

  //1st task: avoid collisions
  y_target = ARR(.0);
  y = ARR(S.kinematicsContacts());
  S.jacobianContacts(J);

  Phi  =( (y - y_target)/1e-0 );
  PhiJ =( J / 1e-0 );

  //2nd task: move right hand to goal ONLY AT THE FINAL TIME STEP
  if(t==T){
    y_target = ARR(-0.2, -0.4, 1.1); 
    S.kinematicsPos(y,"handL");
    S.jacobianPos  (J,"handL");

    Phi .append( (y - y_target)/1e-1 );
    PhiJ.append( J / 1e-1 );
  }
}

void TrajectoryOptimizationProblem::getPsi(arr& Psi, arr& PsiA, arr& PsiB, const arr& qt_1, const arr& qt, uint t){
  Psi = qt - qt_1;
  PsiA.setDiag(-1., qt.N);  //A is negative identity matrix
  PsiB.setDiag( 1., qt.N);  //B is identity matrix
}

void TrajectoryOptimizationProblem::getW(arr& W, const arr& qt_1, const arr& qt, uint t){
  W.setDiag(10.,qt.N);
}


//===========================================================================
//
// simplest gradient optimizer
//

void optimize(){
  TrajectoryOptimizationProblem problem;
  uint t,T,k;
  arr q,q_test,grad;
  double cost,cost_test;
  double alpha = 1e-2;
  T=problem.T;

  //stupid trajectory initialization: setting it constant
  //(a trajectory is a matrix, each row is a joint angle vector, has T+1 rows)
  q.resize(T+1, problem.q0.N);
  for(t=0;t<=T;t++) q[t]=problem.q0;

  for(k=0;k<10000;k++){
    //display the trajectory
    for(t=0;t<=T;t++) problem.S.setJointAngles(q[t]);

    //get the gradient
    cost = problem.costAndGradient(&grad,q);
    cout <<"iteration " <<k <<"  cost " <<cost <<"  alpha " <<alpha <<endl;
    gnuplot("plot 'cost' us 1:2 title 'task costs', 'cost' us 1:3 title 'control costs'");

    //simplest Gradient Descent:
    q_test = q - alpha*grad;
    q_test[0] = problem.q0; //fix the start point of the trajectory
    cost_test = problem.costAndGradient(NULL,q_test);

    if(cost_test < cost){
      q = q_test;
      cost=cost_test;
      alpha *= 1.2;
    }else{
      alpha *= 0.5;
    }
  }

  MT::wait();
}



int main(int argc, char **argv){
  optimize();
}
