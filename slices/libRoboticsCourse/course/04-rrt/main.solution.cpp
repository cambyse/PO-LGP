#include <stdlib.h>
#include <MT/roboticsCourse.h>
#include <MT/ann.h>
#include <MT/plot.h>
#include <MT/optimization.h>

struct TrajectoryOptimizationProblem:VectorChainFunction {
  Simulator *S;
  arr x0,xT;
  
  TrajectoryOptimizationProblem(Simulator& _S, uint _T, const arr& _x0, const arr& _xT){
    S = &_S;
    T=_T;
    x0=_x0;
    xT=_xT;
  }
  
  virtual void fvi (arr& y, arr* J, uint i, const arr& x_i){ //tasks: only collision
    S->setJointAngles(x_i,false);
    S->kinematicsContacts(y);
    if(J) S->jacobianContacts(*J);
    double rho=1e3;
    if(i==0){
      y.append(rho*(x_i-x0));
      if(J) J->append(rho*eye(xT.N));
    }else{
    if(i==T){
      y.append(rho*(x_i-xT));
      if(J) J->append(rho*eye(xT.N));
    }else{
      y.append(zeros(1,xT.N));
      if(J) J->append(zeros(xT.N,xT.N));
    }}
  }
  
  virtual void fvij(arr& y, arr* Ji, arr* Jj, uint i, uint j, const arr& x_i, const arr& x_j){ //transitions: simple distance
    CHECK(i==j-1,"");
    double w=1.;
    y = w*(x_i-x_j);
    if(Ji) Ji->setDiag( w,y.N);
    if(Jj) Jj->setDiag(-w,y.N);
  }
};

struct RRT{
private:
  ANN ann;      //ann stores all points added to the tree in ann.X
  uintA parent; //for each point we also store the index of the parent node
  double stepsize;
  uint last_parent;
  
public:
  RRT(const arr& q0, double _stepsize){
    ann   .append(q0); //append q as the root of the tree
    parent.append(0);    //q has itself as parent
    stepsize = _stepsize;
  }
  double getProposalTowards(arr& q){
    //find NN
    uint k=ann.getNN(q);

    //compute little step
    arr d = q - ann.X[k]; //difference vector between q and nearest neighbor
    double dist = norm(d);
    q = ann.X[k] + stepsize/dist * d;
    last_parent = k;
    return dist;
  }
  void add(const arr& q){
    ann.append(q);
    parent.append(last_parent);
  }
  void addLineDraw(const arr& q, Simulator& S){
    //I can't draw the edge in the 7-dim joint space!
    //But I can draw a projected edge in 3D endeffector position space:
    arr y_from,y_to;
    arr line;
    S.setJointAngles(ann.X[last_parent],false);  S.kinematicsPos(y_from,"peg");
    S.setJointAngles(q                 ,false);  S.kinematicsPos(y_to  ,"peg");
    line.append(y_from); line.reshape(1,line.N);
    line.append(y_to);
    plotLine(line); //add a line to the plot

  }
  //some access routines
  uint getParent(uint i){ return parent(i); }
  uint getNumberNodes(){ return ann.X.d0; }
  arr& getNode(uint i){ return ann.X[i](); }
  void getRandomNode(arr& q){ q = ann.X[rnd(ann.X.d0)]; }
};

void RTTplan(){
  Simulator S("../02-pegInAHole/pegInAHole.ors");
  S.setContactMargin(.02); //this is 2 cm (all units are in meter)
  
  arr qT = ARRAY(0.945499, 0.431195, -1.97155, 0.623969, 2.22355, -0.665206, -1.48356);
  arr q0, y_col, q;
  S.getJointAngles(q0);
  q=q0;

  S.setJointAngles(qT);

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  S.watch();        //pause and watch initial posture

  double stepsize = .1;
  RRT rrt0(q0, stepsize);
  RRT rrtT(qT, stepsize);
  
  plotModule.colors=false;
  bool success=false;
  uint success_node0,success_nodeT;
  
  for(uint i=0;i<100000;i++){
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
        success_node0 = rrt0.last_parent;
        success_nodeT = rrtT.last_parent;
      }
    }

    //let rrtT grow
    if(rnd.uni()<.5) rndUniform(q,-MT_2PI,MT_2PI,false);
    else rrt0.getRandomNode(q);
    rrtT.getProposalTowards(q);
    S.setJointAngles(q,false);
    S.kinematicsContacts(y_col);
    cout <<y_col(0) <<endl;
    if(y_col(0)<.5){
      rrtT.add(q);
      rrtT.addLineDraw(q,S);
      //check if we're close to the other tree
      double d=rrt0.getProposalTowards(q);
      if(d<stepsize){
        success = true;
        success_node0 = rrt0.last_parent;
        success_nodeT = rrtT.last_parent;
      }
    }
    
    //some output
    if(!(i%1000)) S.setJointAngles(q); //updates diplay (makes it slow)
    cout <<"\rRRT sizes = " <<rrt0.ann.X.d0  <<' ' <<rrtT.ann.X.d0 <<std::flush;
    
    if(success) break;
  }

  if(!success) return;

  cout <<"SUCCESS!" <<endl;
  uintA nodes0,nodesT;
  
  for(;;){
    nodes0.append(success_node0);
    if(!success_node0) break;
    success_node0 = rrt0.parent(success_node0);
  }
  for(;;){
    nodesT.append(success_nodeT);
    if(!success_nodeT) break;
    success_nodeT = rrtT.parent(success_nodeT);
  }
  
  q.clear();
  for(uint t=nodes0.N;t--;){
    q.append(rrt0.ann.X[nodes0(t)]);
  }
  for(uint t=0;t<nodesT.N;t++){
    q.append(rrtT.ann.X[nodesT(t)]);
  }
  
  //display
  uint n=q0.N;
  q.reshape(q.N/n, n);
  for(uint t=0;t<q.d0;t++){
    S.setJointAngles(q[t], true);
  }
  
  MT::save(q,"q.rrt");
}

void optim(){
  Simulator S("../02-pinInAHole/pin_in_a_hole.ors");
  S.setContactMargin(.02); //this is 2 cm (all units are in meter)
  
  arr x,x0;
  MT::load(x0,"q.rrt");
  x=x0;
  uint T=x.d0-1;
  
  TrajectoryOptimizationProblem P(S, T, x[0], x[T]);
  conv_VectorChainFunction P2(P);
  
  // optimize
  //checkGradient((VectorFunction&)P2, x, 1e-4);

  //eval_cost=0;  x=x0;  optRprop(x, P2, .1, NULL, 1e-3, 1000, 1);  cout <<"-- evals=" <<eval_cost <<endl;
  //eval_cost=0;  x=x0;  optGradDescent(x, P2, .1, NULL, 1e-3, 1000, -1., 1);  cout <<"-- evals=" <<eval_cost <<endl;
  //eval_cost=0;  x=x0;  optGaussNewton(x, P2, NULL, 1e-3, 1000, -1., 1);  cout <<"-- evals=" <<eval_cost <<endl;
  eval_cost=0;  x=x0;  optDynamicProgramming(x, P2, NULL, 1e-3, 1e-4, 100, -1., 2 );  cout <<"-- evals=" <<eval_cost <<endl;
  eval_cost=0;  x=x0;  optMinSumGaussNewton(x, P2, NULL, 1e-3, 1e-4, 100, -1., 2 );  cout <<"-- evals=" <<eval_cost <<endl;
  //eval_cost=0;    optNodewise(x, P, NULL, 1e-3, 1000, -1., 2);  cout <<"-- evals=" <<eval_cost <<endl;

  //display
  for(uint t=0;t<=T;t++) S.setJointAngles(x[t], true);
  S.watch();
  for(uint t=0;t<=T;t++) S.setJointAngles(x[t], true);
  S.watch();
}

int main(int argc,char **argv){
  //RTTplan();
  optim();

  return 0;
}
