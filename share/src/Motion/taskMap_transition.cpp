#include "taskMap_transition.h"
#include "taskMap_qItself.h"

TransitionTaskMap::TransitionTaskMap(const ors::KinematicWorld& G, bool fixJointsOnly)
  : fixJointsOnly(fixJointsOnly){
  posCoeff = mlr::getParameter<double>("Motion/TaskMapTransition/posCoeff",.0);
  velCoeff = mlr::getParameter<double>("Motion/TaskMapTransition/velCoeff",.0);
  accCoeff = mlr::getParameter<double>("Motion/TaskMapTransition/accCoeff",1.);

  //transition cost metric
  H_rate = mlr::getParameter<double>("Hrate");
  arr H_diag;
  if(mlr::checkParameter<arr>("Hdiag")) {
    H_diag = mlr::getParameter<arr>("Hdiag");
  } else {
    H_diag = G.getHmetric(); //G.naturalQmetric();
  }
  H_rate_diag = H_rate*H_diag;
}

uint TransitionTaskMap::dim_phi(const WorldL& G, int t){
  bool handleSwitches=fixJointsOnly;
  uint qN=G(0)->q.N;
  for(uint i=0;i<G.N;i++) if(G(i)->q.N!=qN){ handleSwitches=true; break; }
//  handleSwitches=true;

  if(!handleSwitches){
    return G.last()->getJointStateDimension();
  }else{
//    for(uint i=0;i<G.N;i++) cout <<i <<' ' <<G(i)->joints.N <<' ' <<G(i)->q.N <<' ' <<G(i)->getJointStateDimension() <<endl;
    mlr::Array<ors::Joint*> matchingJoints = getMatchingJoints(G.sub(-1-order,-1), fixJointsOnly);
    uint ydim=0;
    for(uint i=0;i<matchingJoints.d0;i++){
//      cout <<i <<' ' <<matchingJoints(i,0)->qIndex <<' ' <<matchingJoints(i,0)->qDim() <<' ' <<matchingJoints(i,0)->name <<endl;
      ydim += matchingJoints(i,0)->qDim();
    }
    return ydim;
  }
  return uint(-1);
}

void TransitionTaskMap::phi(arr& y, arr& J, const WorldL& G, double tau, int t){
  if(G.last()->q_agent!=0){ //we're referring to a graph set to non-zero agent!
    uint n=G.last()->getJointStateDimension();
    CHECK(n!=H_rate_diag.N,"just checking...");
    y.resize(n).setZero();
    if(&J) J.resize(y.N, order+1, n).setZero();
    return;
  }

  bool handleSwitches=fixJointsOnly;
  uint qN=G(0)->q.N;
  for(uint i=0;i<G.N;i++) if(G(i)->q.N!=qN){ handleSwitches=true; break; }
//  handleSwitches=true;

  if(!handleSwitches){ //simple implementation
    //-- transition costs
    double h = H_rate*sqrt(tau), tau2=tau*tau;
//    arr h = sqrt(H_rate_diag)*sqrt(tau);
    y.resize(G.last()->q.N).setZero();
    if(order==1) velCoeff = 1.;
    if(order>=0 && posCoeff) y += posCoeff*(G(G.N-1-0)->q); //penalize position
    if(order>=1 && velCoeff) y += velCoeff*(G(G.N-1-1)->q - G(G.N-1-0)->q)/tau; //penalize velocity
    if(order>=2 && accCoeff) y += accCoeff*(G(G.N-1-2)->q - 2.*G(G.N-1-1)->q + G(G.N-1-0)->q)/tau2; //penalize acceleration
    if(order>=3) NIY; //  y = (x_bar[3]-3.*x_bar[2]+3.*x_bar[1]-x_bar[0])/tau3; //penalize jerk
    //multiply with h...
//    y = h % y;
    for(ors::Joint *j:G.last()->joints) for(uint i=0;i<j->qDim();i++)
      y(j->qIndex+i) *= h*j->H;

    if(&J) {
      uint n = G.last()->q.N;
      J.resize(y.N, G.N, n).setZero();
      for(uint i=0;i<n;i++){
        if(order>=0 && posCoeff){ J(i,G.N-1-0,i) += posCoeff; }
        if(order>=1 && velCoeff){ J(i,G.N-1-1,i) += velCoeff/tau;  J(i,G.N-1-0,i) += -velCoeff/tau; }
        if(order>=2 && accCoeff){ J(i,G.N-1-2,i) += accCoeff/tau2;  J(i,G.N-1-1,i) += -2.*accCoeff/tau2;  J(i,G.N-1-0,i) += accCoeff/tau2; }
        //      if(order>=3){ J(i,3,i) = 1.;  J(i,2,i) = -3.;  J(i,1,i) = +3.;  J(i,0,i) = -1.; }
      }
      J.reshape(y.N, G.N*n);
//      for(uint i=0; i<n; i++) J[i]() *= h(i);
      for(ors::Joint *j:G.last()->joints) for(uint i=0;i<j->qDim();i++)
        J[j->qIndex+i] *= h*j->H;
    }
  }else{ //with switches
    mlr::Array<ors::Joint*> matchingJoints = getMatchingJoints(G.sub(-1-order,-1), fixJointsOnly);
    double h = H_rate*sqrt(tau), tau2=tau*tau;

//    getSwitchedJoints(*G.elem(-2), *G.elem(-1), true);

    uint ydim=0;
    uintA qidx(G.N);
    for(uint i=0;i<matchingJoints.d0;i++) ydim += matchingJoints(i,0)->qDim();
    y.resize(ydim).setZero();
    if(&J) {
      qidx(0)=0;
      for(uint i=1;i<G.N;i++) qidx(i) = qidx(i-1)+G(i-1)->q.N;
      J.resize(ydim, qidx.last()+G.last()->q.N).setZero();
    }

    uint m=0;
    for(uint i=0;i<matchingJoints.d0;i++){
      mlr::Array<ors::Joint*> joints = matchingJoints[i];
      uint jdim = joints(0)->qDim(), qi1, qi2, qi3;
      for(uint j=0;j<jdim;j++){
        if(order>=0) qi1 = joints.elem(-1)->qIndex+j;
        if(order>=1) qi2 = joints.elem(-2)->qIndex+j;
        if(order>=2 && accCoeff) qi3 = joints.elem(-3)->qIndex+j;
        double hj = h * joints.last()->H;
        //TODO: adding vels + accs before squareing does not make much sense?
        if(order>=0 && posCoeff) y(m) += posCoeff*hj*( (G.elem(-1)->q(qi1)));
        if(order>=1 && velCoeff) y(m) += velCoeff*hj*( (G.elem(-1)->q(qi1) -    G.elem(-2)->q(qi2)) /tau);
        if(order>=2 && accCoeff) y(m) += accCoeff*hj*( (G.elem(-1)->q(qi1) - 2.*G.elem(-2)->q(qi2) + G.elem(-3)->q(qi3)) /tau2);
        if(&J){
          if(order>=0 && posCoeff){ J(m, qidx.elem(-1)+qi1) += posCoeff*hj; }
          if(order>=1 && velCoeff){ J(m, qidx.elem(-1)+qi1) += velCoeff*hj/tau;  J(m, qidx.elem(-2)+qi2) += -velCoeff*hj/tau; }
          if(order>=2 && accCoeff){ J(m, qidx.elem(-1)+qi1) += accCoeff*hj/tau2; J(m, qidx.elem(-2)+qi2) += -2.*accCoeff*hj/tau2; J(m, qidx.elem(-3)+qi3) += accCoeff*hj/tau2; }
        }
        m++;
      }
    }
    CHECK(m==ydim,"");
  }
}

