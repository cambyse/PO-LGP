#include "taskMap_default.h"

DefaultTaskMap::DefaultTaskMap(DefaultTaskMapType _type,
                               int iShape, const ors::Vector& _ivec,
                               int jShape, const ors::Vector& _jvec,
                               const arr& _params):type(_type), i(iShape), j(jShape){
  if(&_ivec) ivec = _ivec;
  if(&_jvec) jvec = _jvec;
  if(&_params) params=_params;
}

DefaultTaskMap::DefaultTaskMap(DefaultTaskMapType _type, const ors::Graph &G,
                               const char* iShapeName, const ors::Vector& _ivec,
                               const char* jShapeName, const ors::Vector& _jvec,
                               const arr& _params):type(_type), i(-1), j(-1){
  ors::Shape *a = iShapeName ? G.getShapeByName(iShapeName):NULL;
  ors::Shape *b = jShapeName ? G.getShapeByName(jShapeName):NULL;
  if(a) i=a->index;
  if(b) j=b->index;
  if(&_ivec) ivec = _ivec;
  if(&_jvec) jvec = _jvec;
  if(&_params) params=_params;
}


void DefaultTaskMap::phi(arr& y, arr& J, const ors::Graph& G) {
  int body_i = i<0?-1: G.shapes(i)->body->index;
  int body_j = j<0?-1: G.shapes(j)->body->index;
  ors::Vector vec_i = i<0?ivec: G.shapes(i)->rel*ivec;
  ors::Vector vec_j = j<0?jvec: G.shapes(j)->rel*jvec;

  arr q;
  ors::Vector pi, pj, c;
  arr zi, zj, Ji, Jj, JRj;
  ors::Transformation f, fi, fj;
  ors::Vector vi, vj, r, jk;
  uint k,l;


  //get state
  switch(type) {
    case posTMT:
      if(body_j==-1) {
        G.kinematicsPos(y, body_i, &vec_i);
        if(&J) G.jacobianPos(J, body_i, &vec_i);
        break;
      }
      pi = G.bodies(body_i)->X * vec_i;
      pj = G.bodies(body_j)->X * vec_j;
      c = G.bodies(body_j)->X.rot / (pi-pj);
      y = ARRAY(c);
      if(&J) {
        G.jacobianPos(Ji, body_i, &vec_i);
        G.jacobianPos(Jj, body_j, &vec_j);
        G.jacobianR(JRj, body_j);
        J.resize(3, Jj.d1);
        for(k=0; k<Jj.d1; k++) {
          vi.set(Ji(0, k), Ji(1, k), Ji(2, k));
          vj.set(Jj(0, k), Jj(1, k), Jj(2, k));
          r .set(JRj(0, k), JRj(1, k), JRj(2, k));
          jk =  G.bodies(body_j)->X.rot / (vi - vj);
          jk -= G.bodies(body_j)->X.rot / (r ^(pi - pj));
          J(0, k)=jk.x; J(1, k)=jk.y; J(2, k)=jk.z;
        }
      }
      break;
    case vecTMT:
      if(body_j==-1) {
        G.kinematicsVec(y, body_i, &vec_i);
        if(&J) G.jacobianVec(J, body_i, &vec_i);
        break;
      }
      //relative
      MT_MSG("warning - don't have a correct Jacobian for this TMType yet");
//      fi = G.bodies(body_i)->X; fi.appendTransformation(irel);
//      fj = G.bodies(body_j)->X; fj.appendTransformation(jrel);
//      f.setDifference(fi, fj);
//      f.rot.getZ(c);
//      y = ARRAY(c);
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
    case vecAlignTMT:
      G.kinematicsVec(zi, body_i, &vec_i);
      if(&J) G.jacobianVec(Ji, body_i, &vec_i);
      if(body_j==-1) {
        zj = ARRAY(vec_j);
        if(&J) { Jj.resizeAs(Ji); Jj.setZero(); }
      } else {
        G.kinematicsVec(zj, body_j, &vec_j);
        if(&J) G.jacobianVec(Jj, body_j, &vec_j);
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

uint DefaultTaskMap::dim_phi(const ors::Graph& G) {
  //get state
  switch(type) {
    case posTMT: return 3;
    case vecTMT: return 3;
    case qItselfTMT: return G.getJointStateDimension();
    case qLinearTMT: return params.d0;
    case qSquaredTMT: return 1;
    case qSingleTMT: return 1;
    case qLimitsTMT: return 1;
    case comTMT: return 2;
    case collTMT: return 1;
    case colConTMT: return 1;
    case skinTMT: return params.N;
    case vecAlignTMT: return 1;
    default:  HALT("no such TMT");
  }
}
