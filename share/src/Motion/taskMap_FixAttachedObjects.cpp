#include "taskMap_FixAttachedObjects.h"
#include "taskMap_qItself.h"

uint FixAttachedObjectsTaskMap::dim_phi(const WorldL& G, int t){
NIY
}


void FixAttachedObjectsTaskMap::phi(arr& y, arr& J, const WorldL& G, double tau, int t){
//  if(G.last()->q_agent!=0){ //we're referring to a graph set to non-zero agent!
//    uint n=G.last()->getJointStateDimension();
//    CHECK(n!=H_rate_diag.N,"just checking...");
//    y.resize(n).setZero();
//    if(&J) J.resize(y.N, order+1, n).setZero();
//    return;
//  }

//  bool handleSwitches=fixJointsOnly;
//  uint qN=G(0)->q.N;
//  for(uint i=0;i<G.N;i++) if(G(i)->q.N!=qN){ handleSwitches=true; break; }
////  handleSwitches=true;

//  { //with switches
//    mlr::Array<ors::Joint*> matchingJoints = getMatchingJoints(G.sub(-1-order,-1), fixJointsOnly);
//    double h = H_rate*sqrt(tau), tau2=tau*tau;

//    uint ydim=0;
//    uintA qidx(G.N);
//    for(uint i=0;i<matchingJoints.d0;i++) ydim += matchingJoints(i,0)->qDim();
//    y.resize(ydim).setZero();
//    if(&J) {
//      qidx(0)=0;
//      for(uint i=1;i<G.N;i++) qidx(i) = qidx(i-1)+G(i-1)->q.N;
//      J.resize(ydim, qidx.last()+G.last()->q.N).setZero();
//    }

//    uint m=0;
//    for(uint i=0;i<matchingJoints.d0;i++){
//      mlr::Array<ors::Joint*> joints = matchingJoints[i];
//      uint jdim = joints(0)->qDim(), qi1, qi2, qi3;
//      for(uint j=0;j<jdim;j++){
//        if(order>=0) qi1 = joints.elem(-1)->qIndex+j;
//        if(order>=1) qi2 = joints.elem(-2)->qIndex+j;
//        if(order>=2 && accCoeff) qi3 = joints.elem(-3)->qIndex+j;
//        double hj = h * joints.last()->H;
//        //TODO: adding vels + accs before squareing does not make much sense?
//        if(order>=0 && posCoeff) y(m) += posCoeff*hj*( (G.elem(-1)->q(qi1)));
//        if(order>=1 && velCoeff) y(m) += velCoeff*hj*( (G.elem(-1)->q(qi1) -    G.elem(-2)->q(qi2)) /tau);
//        if(order>=2 && accCoeff) y(m) += accCoeff*hj*( (G.elem(-1)->q(qi1) - 2.*G.elem(-2)->q(qi2) + G.elem(-3)->q(qi3)) /tau2);
//        if(&J){
//          if(order>=0 && posCoeff){ J(m, qidx.elem(-1)+qi1) += posCoeff*hj; }
//          if(order>=1 && velCoeff){ J(m, qidx.elem(-1)+qi1) += velCoeff*hj/tau;  J(m, qidx.elem(-2)+qi2) += -velCoeff*hj/tau; }
//          if(order>=2 && accCoeff){ J(m, qidx.elem(-1)+qi1) += accCoeff*hj/tau2; J(m, qidx.elem(-2)+qi2) += -2.*accCoeff*hj/tau2; J(m, qidx.elem(-3)+qi3) += accCoeff*hj/tau2; }
//        }
//        m++;
//      }
//    }
//    CHECK(m==ydim,"");
//  }

}


