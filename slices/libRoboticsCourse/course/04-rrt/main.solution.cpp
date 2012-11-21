#include <stdlib.h>
#include <MT/roboticsCourse.h>
#include <MT/ann.h>
#include <MT/plot.h>
#include <MT/optimization.h>

#include <MT/kOrderMarkovProblem.h>
//#include <MT/functions.h>

struct TrajectoryOptimizationProblem:KOrderMarkovFunction {
  Simulator *S;
  uint T;
  arr x0, xT;
  void phi_t(arr& phi, arr& J, uint t, const arr& x_bar);

  uint get_T(){ return T; }
  uint get_k(){ return 2; }
  uint get_n(){ return S->getJointDimension(); }
  uint get_m(uint t){
    if(t==0 || t==get_T()-get_k()) return 2*get_n();
    return get_n()+1;
  }
};

struct RRT{
private:
  ANN ann;      //ann stores all points added to the tree in ann.X
  uintA parent; //for each point we also store the index of the parent node
  double stepsize;
  uint nearest;
  
public:
  RRT(const arr& q0, double _stepsize){
    ann   .append(q0); //append q as the root of the tree
    parent.append(0);    //q has itself as parent
    stepsize = _stepsize;
  }
  double getProposalTowards(arr& q){
    //find NN
    nearest=ann.getNN(q);

    //compute little step
    arr d = q - ann.X[nearest]; //difference vector between q and nearest neighbor
    double dist = norm(d);
    q = ann.X[nearest] + stepsize/dist * d;
    return dist;
  }
  void add(const arr& q){
    ann.append(q);
    parent.append(nearest);
  }
  void addLineDraw(const arr& q, Simulator& S){
    //I can't draw the edge in the 7-dim joint space!
    //But I can draw a projected edge in 3D endeffector position space:
    arr y_from,y_to;
    arr line;
    S.setJointAngles(ann.X[nearest],false);  S.kinematicsPos(y_from,"peg");
    S.setJointAngles(q                 ,false);  S.kinematicsPos(y_to  ,"peg");
    line.append(y_from); line.reshape(1,line.N);
    line.append(y_to);
    plotLine(line); //add a line to the plot

  }
  //some access routines
  uint getNearest(){ return nearest; }
  uint getParent(uint i){ return parent(i); }
  uint getNumberNodes(){ return ann.X.d0; }
  arr getNode(uint i){ return ann.X[i]; }
  void getRandomNode(arr& q){ q = ann.X[rnd(ann.X.d0)]; }
};

void plotEffTraj(Simulator &S, const arr& q){
  arr y,line;
  for(uint t=0;t<q.d0;t++){
    S.setJointAngles(q[t],false);
    S.kinematicsPos(y,"peg");
    line.append(y);
  }
  line.reshape(line.N/y.N,y.N);
  plotLine(line);
}

void RTTplan(){
  Simulator S("../02-pegInAHole/pegInAHole.ors");
  S.setContactMargin(.02); //this is 2 cm (all units are in meter)
  
  arr qT = ARRAY(0.945499, 0.431195, -1.97155, 0.623969, 2.22355, -0.665206, -1.48356);
  arr q0, y_col, q;
  S.getJointAngles(q0);
  q=q0;

  S.setJointAngles(qT);

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  //S.watch();        //pause and watch initial posture

  double stepsize = .1;
  RRT rrt0(q0, stepsize);
  RRT rrtT(qT, stepsize);
  
  plotModule.colors=false;
  bool success=false;
  uint success_node0,success_nodeT;
  
  uint i;
  for(i=0;i<100000;i++){
    //let rrt0 grow
    if(rnd.uni()<.5) rndUniform(q,-MT_2PI,MT_2PI,false);
    else rrtT.getRandomNode(q);
    rrt0.getProposalTowards(q);
    S.setJointAngles(q,false);
    S.kinematicsContacts(y_col);
    if(y_col(0)<.5){
      rrt0.add(q);
      rrt0.addLineDraw(q,S);
      //check if we're close to the other tree
      double d=rrtT.getProposalTowards(q);
      if(d<stepsize){
        success = true;
        success_node0 = rrt0.getNearest();
        success_nodeT = rrtT.getNearest();
      }
    }

    //let rrtT grow
    if(rnd.uni()<.5) rndUniform(q,-MT_2PI,MT_2PI,false);
    else rrt0.getRandomNode(q);
    rrtT.getProposalTowards(q);
    S.setJointAngles(q,false);
    S.kinematicsContacts(y_col);
    if(y_col(0)<.5){
      rrtT.add(q);
      rrtT.addLineDraw(q,S);
      //check if we're close to the other tree
      double d=rrt0.getProposalTowards(q);
      if(d<stepsize){
        success = true;
        success_node0 = rrt0.getNearest();
        success_nodeT = rrtT.getNearest();
      }
    }
    
    //some output
    if(!(i%1000)){
      S.setJointAngles(q); //updates display (makes it slow)
      cout <<"\rRRT sizes = " <<rrt0.getNumberNodes()  <<' ' <<rrtT.getNumberNodes() <<std::flush;
    }
    
    if(success) break;
  }

  if(!success) return;

  cout <<"\nSUCCESS!"
      <<"\n  tested samples=" <<2*i
      <<"\n  #fwd-tree-size=" <<rrt0.getNumberNodes()
     <<"\n  #bwd-tree-size=" <<rrtT.getNumberNodes()
       <<endl;

  uintA nodes0,nodesT;
  for(;;){
    nodes0.append(success_node0);
    if(!success_node0) break;
    success_node0 = rrt0.getParent(success_node0);
  }
  for(;;){
    nodesT.append(success_nodeT);
    if(!success_nodeT) break;
    success_nodeT = rrtT.getParent(success_nodeT);
  }
  
  q.clear();
  for(uint t=nodes0.N;t--;){
    q.append(rrt0.getNode(nodes0(t)));
  }
  for(uint t=0;t<nodesT.N;t++){
    q.append(rrtT.getNode(nodesT(t)));
  }
  
  //display
  uint n=q0.N;
  q.reshape(q.N/n, n);
  cout <<"  path-length=" <<q.d0 <<endl;
  for(uint t=0;t<q.d0;t++){
    S.setJointAngles(q[t], true);
  }

  MT::save(q,"q.rrt");
}

void optim(){
  Simulator S("../02-pegInAHole/pegInAHole.ors");
  S.setContactMargin(.01); //this is 2 cm (all units are in meter)
  
  arr x,x0;
  MT::load(x0,"q.rrt");
  x=x0;
  plotClear();
  plotEffTraj(S, x);
  for(uint t=0;t<x.d0;t++) S.setJointAngles(x[t], true);
  //S.watch();

  TrajectoryOptimizationProblem P;
  P.S=&S;
  P.T=x.d0-1;
  P.x0 = x[0];
  P.xT = x[x.d0-1];

  cout <<"Problem parameters:"
       <<"\n T=" <<P.get_T()
       <<"\n k=" <<P.get_k()
       <<"\n n=" <<P.get_n()
       <<endl;

#if 0 //only if you want to see some steps...
  for(uint k=0;k<20;k++){
    optGaussNewton(x, Convert(P), OPT5(stopIters=1, verbose=2, useAdaptiveDamping=.0, maxStep=.1, stopTolerance=1e-2));
    plotEffTraj(S, x);
    S.watch();
  }
#endif

  optGaussNewton(x, Convert(P), OPT5(stopIters=1000, verbose=2, useAdaptiveDamping=.0, maxStep=.1, stopTolerance=1e-2));
  MT::save(x,"q.optim");

  //display
  plotEffTraj(S, x);
  for(;;){
    for(uint t=0;t<=P.get_T();t++) S.setJointAngles(x[t], true);
    S.watch();
  }
}

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  switch(MT::getParameter<int>("mode",0)){
  case 0: RTTplan(); //break;
  case 1: optim(); break;
  }

  return 0;
}


void TrajectoryOptimizationProblem::phi_t(arr& phi, arr& J, uint t, const arr& x_bar){
  uint T=get_T(), n=get_n(), k=get_k(), m=get_m(t);

  //assert some dimensions
  CHECK(x_bar.d0==k+1,"");
  CHECK(x_bar.d1==n,"");
  CHECK(t<=T-k,"");

  phi.resize(m);
  phi.setZero();

  //curvature
  if(k==1)  phi.setVectorBlock(x_bar[1]-x_bar[0], 0); //penalize velocity
  if(k==2)  phi.setVectorBlock(x_bar[2]-2.*x_bar[1]+x_bar[0], 0); //penalize acceleration
  if(k==3)  phi.setVectorBlock(x_bar[3]-3.*x_bar[2]+3.*x_bar[1]-x_bar[0], 0); //penalize jerk

  //task
  if(t==0){
    phi.setVectorBlock(1e2*(x_bar[0]-x0),m-n);
  }else if(t==get_T()-get_k()){
    phi.setVectorBlock(1e2*(x_bar[k]-xT),m-n);
  }else{
    S->setJointAngles(x_bar[1],false);
    arr y_col,J_col;
    S->kinematicsContacts(y_col);
    phi(m-1) = y_col(0);
  }

  if(&J){ //we also need to return the Jacobian
    J.resize(m,k+1,n);
    J.setZero();

    //curvature
    for(uint i=0;i<n;i++){
      if(k==1){ J(i,1,i) = 1.;  J(i,0,i) = -1.; }
      if(k==2){ J(i,2,i) = 1.;  J(i,1,i) = -2.;  J(i,0,i) = 1.; }
      if(k==3){ J(i,3,i) = 1.;  J(i,2,i) = -3.;  J(i,1,i) = +3.;  J(i,0,i) = -1.; }
    }

    //task
    if(t==0){
      for(uint i=0;i<n;i++) J(n+i,0,i) = 1e2;
    }else if(t==get_T()-get_k()){
      for(uint i=0;i<n;i++) J(n+i,k,i) = 1e2;
    }else{
      arr J_col;
      S->jacobianContacts(J_col);
      for(uint i=0;i<n;i++){
        J(m-1,1,i) = J_col(0,i);
      }
    }
  }
}
