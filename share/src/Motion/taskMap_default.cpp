#include "taskMap_default.h"

void DefaultTaskMap::phi(arr& y, arr& J, const ors::Graph& G) {
  arr q;
  ors::Vector pi, pj, c;
  arr zi, zj, Ji, Jj, JRj;
  ors::Transformation f, fi, fj;
  ors::Vector vi, vj, r, jk;
  uint k,l;

  //get state
  switch(type) {
    case posTMT:
      if(j==-1) {
        G.kinematicsPos(y, i, &irel.pos);
        if(&J) G.jacobianPos(J, i, &irel.pos);
        break;
      }
      pi = G.bodies(i)->X.pos + G.bodies(i)->X.rot * irel.pos;
      pj = G.bodies(j)->X.pos + G.bodies(j)->X.rot * jrel.pos;
      c = G.bodies(j)->X.rot / (pi-pj);
      y.resize(3); y = ARRAY(c);
      G.jacobianPos(Ji, i, &irel.pos);
      G.jacobianPos(Jj, j, &jrel.pos);
      G.jacobianR(JRj, j);
      if(&J) {
        J.resize(3, Jj.d1);
        for(k=0; k<Jj.d1; k++) {
          vi.set(Ji(0, k), Ji(1, k), Ji(2, k));
          vj.set(Jj(0, k), Jj(1, k), Jj(2, k));
          r .set(JRj(0, k), JRj(1, k), JRj(2, k));
          jk =  G.bodies(j)->X.rot / (vi - vj);
          jk -= G.bodies(j)->X.rot / (r ^(pi - pj));
          J(0, k)=jk.x; J(1, k)=jk.y; J(2, k)=jk.z;
        }
      }
      break;
    case zoriTMT:
      if(j==-1) {
        G.kinematicsVec(y, i, &irel.rot.getZ(vi));
        if(&J) G.jacobianVec(J, i, &irel.rot.getZ(vi));
        break;
      }
      //relative
      MT_MSG("warning - don't have a correct Jacobian for this TMType yet");
      fi = G.bodies(i)->X; fi.appendTransformation(irel);
      fj = G.bodies(j)->X; fj.appendTransformation(jrel);
      f.setDifference(fi, fj);
      f.rot.getZ(c);
      y = ARRAY(c);
      NIY; //TODO: Jacobian?
      break;
    case qItselfTMT:   G.getJointState(q);    y = q;   if(&J) J.setId(q.N);  break;
    case qLinearTMT:   G.getJointState(q);    y = params * q;   if(&J) J=params;  break;
    case qSquaredTMT:
      G.getJointState(q);
      y.resize(1);  y(0) = scalarProduct(params, q, q);
      if(&J) {
        J = params * q;
        J *= (double)2.;
        J.reshape(1, q.N);
      }
      break;
    case qSingleTMT:
      G.getJointState(q);
      y.resize(1);  y(0)=q(-i);
      if(&J) {
        J.resize(1, G.getJointStateDimension());
        J.setZero();
        J(0, -i) = 1.;
      }
      break;
    case qLimitsTMT:   G.getLimitsMeasure(y, params);  if(&J) G.getLimitsGradient(J, params);   break;
    case comTMT:       G.getCenterOfMass(y);     y.resizeCopy(2); if(&J) { G.getComGradient(J);  J.resizeCopy(2, J.d1); }  break;
    case collTMT:      G.phiCollision(y, J, params(0));  break;
    case colConTMT:    G.getContactConstraints(y);  if(&J) G.getContactConstraintsGradient(J); break;
    case skinTMT:
      y.resize(params.N);
      y.setZero();
      if(&J) {
        J.clear();
        for(k=0; k<params.N; k++) {
          l=(uint)params(k);
          G.jacobianPos(Ji, l, NULL);
          G.bodies(l)->X.rot.getY(vi);
          vi *= -1.;
          zi = ARRAY(vi);
          J.append(~zi*Ji);
        }
        J.reshape(params.N, J.N/params.N);
      }
      break;
    case zalignTMT:
      G.kinematicsVec(zi, i, &irel.rot.getZ(vi));
      if(&J) G.jacobianVec(Ji, i, &irel.rot.getZ(vi));
      if(j==-1) {
        ors::Vector world_z;
        if(params.N==3) world_z.set(params.p); else world_z=Vector_z;
        zj = ARRAY((jrel*world_z));
        if(&J) { Jj.resizeAs(Ji); Jj.setZero(); }
      } else {
        G.kinematicsVec(zj, j, &jrel.rot.getZ(vj));
        if(&J) G.jacobianVec(Jj, j, &jrel.rot.getZ(vj));
      }
      y.resize(1);
      y(0) = scalarProduct(zi, zj);
      if(&J) {
        J = ~zj * Ji + ~zi * Jj;
        J.reshape(1, G.getJointStateDimension());
      }
      break;
    default:  HALT("no such TVT");
  }
}

uint DefaultTaskMap::phiDim(const ors::Graph& G) {
  //get state
  switch(type) {
    case posTMT: return 3;
    case zoriTMT: return 3;
    case qItselfTMT: return G.getJointStateDimension();
    case qLinearTMT: return params.d0;
    case qSquaredTMT: return 1;
    case qSingleTMT: return 1;
    case qLimitsTMT: return 1;
    case comTMT: return 2;
    case collTMT: return 1;
    case colConTMT: return 1;
    case skinTMT: return params.N;
    case zalignTMT: return 1;
    default:  HALT("no such TVT");
  }
}
