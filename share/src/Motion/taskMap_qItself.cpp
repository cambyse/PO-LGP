#include <climits>
#include "taskMap_qItself.h"

void TaskMap_qItself::phi(arr& q, arr& J, const ors::KinematicWorld& G, int t) {
  G.getJointState(q);
  if(M.N){
    if(M.nd==1){
      q=M%q; if(&J) J.setDiag(M);
    }else{
      q=M*q; if(&J) J=M;
    }
  }else{
    if(&J) J.setId(q.N);
  }
}

void TaskMap_qItself::phi(arr& y, arr& J, const WorldL& G, double tau, int t){
  CHECK(G.N>=order+1,"I need at least " <<order+1 <<" configurations to evaluate");
  uint k=order;
  if(k==0) return TaskMap::phi(y, J, G, tau, t);

  double tau2=tau*tau, tau3=tau2*tau;
  arrA q_bar(k+1), J_bar(k+1);
  //-- read out the task variable from the k+1 configurations
  uint offset = G.N-1-k; //G.N might contain more configurations than the order of THIS particular task -> the front ones are not used
  for(uint i=0;i<=k;i++){
    q_bar(i) = G(offset+i)->q;
    J_bar(i).setId(q_bar(i).N);
  }
  bool handleSwitches=false;
  uint qN=q_bar(0).N;
  for(uint i=0;i<=k;i++) if(q_bar(i).N!=qN){ handleSwitches=true; break; }
  if(handleSwitches){
    uint nJoints = G(offset)->joints.N;
    JointL jointMatchLists(k+1, nJoints); //for each joint of [0], find if the others have it
    jointMatchLists.setZero();
    boolA useIt(nJoints);
    useIt = true;
    for(uint j_idx=0; j_idx<nJoints; j_idx++){
      ors::Joint *j=G(offset)->joints(j_idx);
      for(uint i=0;i<=k;i++){
        ors::Joint *jmatch = G(offset+i)->getJointByBodyNames(j->from->name, j->to->name);
        if(!jmatch){ useIt(j_idx) = false; break; }
        CHECK_EQ(j->type, jmatch->type, "");
        jointMatchLists(i, j_idx) = jmatch;
      }
    }

    arrA q_bar_mapped(k+1), J_bar_mapped(k+1);
    uint qidx, qdim;
    for(uint j_idx=0; j_idx<nJoints; j_idx++){
      if(useIt(j_idx)){
        for(uint i=0;i<=k;i++){
          qidx=jointMatchLists(i,j_idx)->qIndex;
          qdim=jointMatchLists(i,j_idx)->qDim();
          q_bar_mapped(i).append(q_bar(i).subRange(qidx, qidx+qdim-1));
          J_bar_mapped(i).append(J_bar(i).subRange(qidx, qidx+qdim-1));
        }
      }
    }

    q_bar = q_bar_mapped;
    J_bar = J_bar_mapped;
  }

  if(k==1)  y = (q_bar(1)-q_bar(0))/tau; //penalize velocity
  if(k==2)  y = (q_bar(2)-2.*q_bar(1)+q_bar(0))/tau2; //penalize acceleration
  if(k==3)  y = (q_bar(3)-3.*q_bar(2)+3.*q_bar(1)-q_bar(0))/tau3; //penalize jerk
  if(&J) {
#if 1
    uintA qidx(G.N);
    qidx(0)=0;
    for(uint i=1;i<G.N;i++) qidx(i) = qidx(i-1)+G(i-1)->q.N;
    J = zeros(y.N, qidx.last()+G.last()->q.N);
    if(k==1){ J.setMatrixBlock(J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(   -J_bar(0), 0, qidx(offset+0));  J/=tau; }
    if(k==2){ J.setMatrixBlock(J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(-2.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(J_bar(0)   , 0, qidx(offset+0));  J/=tau2; }
    if(k==3){ J.setMatrixBlock(J_bar(3), 0, qidx(offset+3));  J.setMatrixBlock(-3.*J_bar(2), 0, qidx(offset+2));  J.setMatrixBlock(3.*J_bar(1), 0, qidx(offset+1));  J.setMatrixBlock(-J_bar(0), 0, qidx(offset+0));  J/=tau3; }
#else
    J = zeros(G.N, y.N, J_bar(0).d1);
    if(k==1){ J[offset+1]() =  J_bar(1);  J[offset+0]() =    -J_bar(0);  J/=tau; }
    if(k==2){ J[offset+2]() =  J_bar(2);  J[offset+1]() = -2.*J_bar(1);  J[offset+0]() = J_bar(0);  J/=tau2; }
    if(k==3){ J[offset+3]() =  J_bar(3);  J[offset+2]() = -3.*J_bar(2);  J[offset+1]() = 3.*J_bar(1);  J[offset+0]() = -J_bar(0);  J/=tau3; }
    arr tmp(J);
    tensorPermutation(J, tmp, TUP(1u,0u,2u));
    J.reshape(y.N, G.N*J_bar(0).d1);
#endif
  }
}


uint TaskMap_qItself::dim_phi(const ors::KinematicWorld& G) {
  if(M.nd==2) return M.d0;
  return G.getJointStateDimension();
}

uint TaskMap_qItself::dim_phi(const WorldL& G, int t){
  if(t<0) return dim_phi(*G.last());

  while(dimPhi.N<=(uint)t) dimPhi.append(UINT_MAX);

  //empirically test the dimension:
  if(dimPhi(t)==UINT_MAX){
    arr y;
    phi(y, NoArr, G, 0.01, t);
    dimPhi(t) = y.N;
  }

  return dimPhi(t);
}
