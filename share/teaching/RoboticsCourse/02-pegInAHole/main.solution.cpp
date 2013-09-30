#include <Ors/roboticsCourse.h>

void sineProfile(arr& q, const arr& q0, const arr& qT,uint T){
  q.resize(T+1,q0.N);
  for(uint t=0; t<=T; t++) q[t] = q0 + .5 * (1.-cos(MT_PI*t/T)) * (qT-q0);
}

void getTaskVector(arr& Phi, arr& PhiJ,  Simulator& S, const arr& pos_target, const arr& dir_target, bool collisions){
  arr y_target,y,J;
  double y_deviation;

  Phi.clear();
  PhiJ.clear();
  
  //1st task: peg positioned within hole
  S.kinematicsPos(y,"peg");
  S.jacobianPos  (J,"peg");
  y_target = pos_target; //exponentially approach the hole
  y_deviation = 1e-2; //(deviation is 1/precision^2)
  Phi .append((y - y_target) / y_deviation);
  PhiJ.append(J / y_deviation);
  
  //2nd task: orientation
  S.kinematicsVec(y, "peg");
  S.jacobianVec(J, "peg");
  y_target = dir_target;
  y_deviation = 1e-2;
  Phi .append((y - y_target) / y_deviation);
  PhiJ.append(J / y_deviation);
  
  //3rd task: collisions
  if(collisions){
    S.kinematicsContacts(y);
    S.jacobianContacts(J);
    y_target = ARR(0.); //target is zero collision costs
    y_deviation = 1e-2;
    Phi .append((y - y_target) / y_deviation);
    PhiJ.append(J / y_deviation);
  }
}

void findPosture(arr& qT, const arr& q0, Simulator& S, uint T, const arr& pos_target, const arr& dir_target, bool collisions){
  arr Phi,PhiJ;
  arr W = eye(S.getJointDimension());
  W /= (double)T;
  
  arr step;
  double max_step_length = .1;
  
  S.setJointAngles(q0);
  qT = q0;
  double cost=-1.,lastCost;

  for(;;){
    getTaskVector(Phi, PhiJ, S, pos_target, dir_target, collisions);

    //evaluate:
    lastCost = cost;
    cost = (~(qT-q0)*W*(qT-q0))(0) + sumOfSqr(Phi);
    cout <<"qT optimization cost = " <<cost <<" [max_step_length=" <<max_step_length <<"]" <<endl;

    //step-size adaptation:
    if(cost>lastCost && lastCost!=-1)  max_step_length *=.5;

    //make a step:
    step = inverse(~PhiJ*PhiJ + W)*(W*(qT-q0) + ~PhiJ* Phi);
    if(norm(step) > max_step_length) step *= max_step_length/norm(step);
    qT -= step;

    //update simulator
    S.setJointAngles(qT);

    if(norm(step)<1e-2) break; //stopping criterion
  }
}


void taskSpaceInterpolation(const arr& q0, Simulator& S, uint T, const arr& pos_target, const arr& dir_target, bool collisions){
  arr q,W,Phi,PhiJ;
  W=eye(S.getJointDimension());

  S.setJointAngles(q0);
  q=q0;
  arr pos0, dir0;
  S.kinematicsPos(pos0,"peg");
  S.kinematicsVec(dir0,"peg");

  for(uint t=0;t<=T;t++){
    getTaskVector(Phi, PhiJ, S,
                  pos0 + .5 * (1.-cos(MT_PI*t/T)) * (pos_target-pos0),
                  dir0 + .5 * (1.-cos(MT_PI*t/T)) * (dir_target-dir0),
                  collisions);

    //compute joint updates
    q -= inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;
    S.setJointAngles(q);
  }
}

void peg_in_a_hole(){
  Simulator S("pegInAHole.ors");
  S.setContactMargin(.02); //this is 2 cm (all units are in meter)

  arr q,q0,q1,q2;
  S.getJointAngles(q0);
  q = q0;
  uint T=100;

  arr pos_hole;
  S.kinematicsPos(pos_hole,"hole");
  
  // a)
  //q1 = pose 30 cm above hole:
  findPosture(q1, q0, S, T, pos_hole+ARR(0,0,.6), ARR(0,0,-1.), false);
  cout <<"optimized q1:" <<q1 <<endl;
  S.watch();

  //q2 = pose in hole:
  findPosture(q2, q1, S, T, pos_hole, ARR(0,0,-1.), false);
  cout <<"optimized q2:" <<q2 <<endl;
  S.watch();
  
  // b)
  cout <<"q-space interpolation:" <<endl;
  for(uint t=0;t<=T;t++){
    q = q0 + .5 * (1.-cos(MT_PI*t/T)) * (q1-q0);
    S.setJointAngles(q);
  }
  for(uint t=0;t<=T;t++){
    q = q1 + .5 * (1.-cos(MT_PI*t/T)) * (q2-q1);
    S.setJointAngles(q);
  }
  S.watch();

  cout <<"y-space interpolation:" <<endl;
  taskSpaceInterpolation(q0, S, T, pos_hole+ARR(0,0,.6), ARR(0,0,-1.), false);
  S.getJointAngles(q);
  taskSpaceInterpolation(q, S, T, pos_hole, ARR(0,0,-1.), true);
  S.watch();

}

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  peg_in_a_hole();

  return 0;
}
