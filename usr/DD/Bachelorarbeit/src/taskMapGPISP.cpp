#include "taskMapGPISP.h"
#include <Motion/taskMap_default.h>

void TaskMap_GPISP::phi(arr& y, arr& J, const ors::KinematicWorld& G, int t) {
  arr pos, JPos;
  posMap.phi(pos, JPos, G);
  arr gradF = gp.gradient(pos);
  y = ARR(gp.evaluate(pos));
  if(&J) {
    J = ~gradF*JPos;
  }
}

TaskMap_GPISP::TaskMap_GPISP(GaussianProcessOptimized& gp, const ors::KinematicWorld& world, const char* shapeName)
  : gp(gp)
  , posMap(posTMT, world, shapeName) {}

//===================================================================================================================

void TaskMap_GPISPNormalOrientation::phi(arr& y, arr& J, const ors::KinematicWorld& G, int t) {
  arr pos, JPos, ori, JOri;
  posMap.phi(pos, JPos, G);
  oriMap.phi(ori, JOri, G);
  arr g = gp.gradient(pos);
  double lg = length(g);

  y = ARR(scalarProduct(ori,g/lg));

  if(&J) {
    arr H = gp.hessian(pos);
    J = ~g/lg*JOri + ~ori*(H*JPos/lg - g*~g*H*JPos/lg/lg/lg);
  }
}

TaskMap_GPISPNormalOrientation::TaskMap_GPISPNormalOrientation(GaussianProcessOptimized& gp, const ors::KinematicWorld& world, const char* shapeName, ors::Vector vector)
  : gp(gp)
  , posMap(posTMT, world, shapeName)
  , oriMap(vecTMT, world, shapeName, vector) {}


//===================================================================================================================


void TaskMap_GPSIPVarianceGeodesic::phi(arr& y, arr& J, const WorldL& G, double tau, int t) {
  arr pos0, pos1, JPos0, JPos1;
  posMap.phi(pos0, JPos0, *G(G.N-1-0));
  posMap.phi(pos1, JPos1, *G(G.N-1-1));
  double v = gp.evaluateVariance((pos0+pos1)/2.0);

  //y = (pos0-pos1)/tau/v;

  y = ARR(length(pos0-pos1)*length(pos0-pos1)/v);

  //cout << G.N << endl;

  if(&J) {
    J = zeros(1,G.N*G.last()->q.N);
    arr gV = gp.gradientVariance((pos0+pos1)/2.0);

    arr JTemp = 2.0*~(pos0-pos1)*JPos0/v-(length(pos0-pos1)*length(pos0-pos1))/v/v/2.0*~gV*JPos0;
    J.setMatrixBlock(JTemp,0,0);

    JTemp = -2.0*~(pos0-pos1)*JPos1/v+(length(pos0-pos1)*length(pos0-pos1))/v/v/2.0*~gV*JPos1;
    J.setMatrixBlock(JTemp,0,G.last()->q.N);

  }

  /*
  if(&J) {
    J = zeros(pos0.N,G.N*G.last()->q.N);
    arr gV = gp.gradientVariance((pos0+pos1)/2.0);

    arr JTemp = JPos0/tau/v - (pos0-pos1)/tau/v/v*~gV*JPos0/2.0;
    J.setMatrixBlock(JTemp,0,0);

    JTemp = -JPos1/tau/v + (pos0-pos1)/tau/v/v*~gV*JPos1/2.0;
    J.setMatrixBlock(JTemp,0,G.last()->q.N);
  }*/
}

TaskMap_GPSIPVarianceGeodesic::TaskMap_GPSIPVarianceGeodesic(GaussianProcessOptimized& gp, const ors::KinematicWorld& world, const char* shapeName)
  : gp(gp)
  , posMap(posTMT, world, shapeName) {}



#if 0
void TaskMapVariance::phi(arr& y, arr& J, const WorldL& G, double tau, int t) {
  y = zeros(1);
  arr v, nonSense, pos0, pos1, pos2, J0, J1, J2;
  taskMap.phi(pos0, J0, *G(G.N-1-0));
  taskMap.phi(pos1, J1, *G(G.N-1-1));
  taskMap.phi(pos2, J2, *G(G.N-1-2));

  gp.evaluate(~pos0, nonSense, v);
  y += v*length((pos0-pos1)/tau)*tau;

  gp.evaluate(~pos1, nonSense, v);
  y += v*length((pos1-pos2)/tau)*tau;

  gp.evaluate(~pos2, nonSense, v);
  y += v*length((pos1-pos2)/tau)*tau;

  /*
  gp.evaluate(~G(G.N-1-0)->q, nonSense, v);
  y += v*length((G(G.N-1-0)->q-G(G.N-1-0-1)->q)/tau)*tau;

  gp.evaluate(~G(G.N-1-1)->q, nonSense, v);
  y += v*length((G(G.N-1-1)->q-G(G.N-1-1-1)->q)/tau)*tau;

  gp.evaluate(~G(G.N-1-2)->q, nonSense, v);
  y += v*length((G(G.N-1-2+1)->q-G(G.N-1-2)->q)/tau)*tau;
  */

  if(&J) {
    J = zeros(1,3*G.last()->q.N);
    arr JTemp;
    arr g;
    gp.gradientV(g,pos2);
    JTemp = (length((pos1-pos2)/tau)*tau*g)*J2;
    J.setMatrixBlock(JTemp,0,0);

    gp.gradientV(g,pos1);
    JTemp = (length((pos1-pos2)/tau)*tau*g)*J1;
    J.setMatrixBlock(JTemp,0,G.last()->q.N);

    gp.gradientV(g,pos0);
    JTemp = (length((pos0-pos1)/tau)*tau*g)*J0;
    J.setMatrixBlock(JTemp,0,2*G.last()->q.N);
  }

#if 0
  if(&J) {
    J = zeros(1,3*G.last()->q.N);
    arr JTemp;
    arr g;
    gp.gradient(g,~pos2);
    JTemp = length((pos1-pos2)/tau)*tau*g;
    J.setMatrixBlock(~JTemp,0,0);

    gp.gradient(g,~pos1);
    JTemp = length((pos1-pos2)/tau)*tau*g;
    J.setMatrixBlock(~JTemp,0,3);

    gp.gradient(g,~pos0);
    JTemp = length((pos0-pos1)/tau)*tau*g;
    J.setMatrixBlock(~JTemp,0,6);
  }
#endif
#if 0
  if(&J) {
    J = zeros(1,3*G.last()->q.N);
    arr JTemp;
    arr g;
    gp.gradient(g,~G(G.N-1-2)->q);
    JTemp = length((G(G.N-1-2+1)->q-G(G.N-1-2)->q)/tau)*tau*g;
    /*gp.evaluate(~G(G.N-1-2)->q, nonSense, v);
      JTemp += -v.first()*tau/length((G(G.N-1-1)->q-G(G.N-1-1-1)->q)/tau)*((G(G.N-1-1)->q-G(G.N-1-1-1)->q))/tau/tau;
      JTemp += v.first()*tau/length((G(G.N-1-1)->q-G(G.N-1-1-1)->q)/tau)*((G(G.N-1-1)->q-G(G.N-1-1-1)->q))/tau/tau;*/
    J.setMatrixBlock(~JTemp,0,0);

    gp.gradient(g,~G(G.N-1-1)->q);
    JTemp = length((G(G.N-1-1)->q-G(G.N-1-1-1)->q)/tau)*tau*g;
    /*gp.evaluate(~G(G.N-1-1)->q, nonSense, v);
      JTemp += -v.first()*tau/length((G(G.N-1-1)->q-G(G.N-1-1-1)->q)/tau)*((G(G.N-1-1)->q-G(G.N-1-1-1)->q))/tau/tau;
      JTemp += v.first()*tau/length((G(G.N-1-1)->q-G(G.N-1-1-1)->q)/tau)*((G(G.N-1-1)->q-G(G.N-1-1-1)->q))/tau/tau;*/
    J.setMatrixBlock(~JTemp,0,3);

    gp.gradient(g,~G(G.N-1-0)->q);
    JTemp = length((G(G.N-1-0)->q-G(G.N-1-0-1)->q)/tau)*tau*g;
    /*gp.evaluate(~G(G.N-1-0)->q, nonSense, v);
      JTemp += -v.first()*tau/length((G(G.N-1-0)->q-G(G.N-1-0-1)->q)/tau)*((G(G.N-1-0)->q-G(G.N-1-0-1)->q))/tau/tau;
      JTemp += v.first()*tau/length((G(G.N-1-0)->q-G(G.N-1-0-1)->q)/tau)*((G(G.N-1-0)->q-G(G.N-1-0-1)->q))/tau/tau;*/
    J.setMatrixBlock(~JTemp,0,6);
  }
#endif
}

TaskMapVariance::TaskMapVariance(GaussianProcess& gp, const ors::KinematicWorld& world, const char* shapeName) : gp(gp), taskMap(posTMT, world, shapeName) {}


//================================================================


void TaskMapGPGradient::phi(arr& y, arr& J, const ors::KinematicWorld& G, int t) {
  arr pos, JacPos;
  positionMap.phi(pos, JacPos, G);
  arr grad, hess;
  gp.gradient(grad, pos);
  gp.hessianPos(hess, pos);

  arr Jac, ori;
  taskMap.phi(ori, Jac, G);

  //y = ori-grad;

  //y = ori/length(ori)-grad/length(grad);

  y = ~ori*grad - ARR(length(ori)*length(grad));
  //cout << grad << endl;

  if(&J) {
    //J = Jac - hess*JacPos;
    //J = Jac/length(ori)-ori*~ori*Jac/(pow(length(ori), 3.0)) - hess*JacPos/length(grad) + grad*~grad*hess*JacPos/(pow(length(grad),3.0));
    J = ~ori*hess*JacPos+~grad*Jac - length(grad)/length(ori)*~ori*Jac-length(ori)/length(grad)*~grad*hess*JacPos;
  }
}

TaskMapGPGradient::TaskMapGPGradient(GaussianProcess& gp, const ors::KinematicWorld& world, const char* shapeName, ors::Vector vector)
  : gp(gp)
  , taskMap(vecTMT, world, shapeName, vector)
  , positionMap(posTMT, world, shapeName) {}


void TaskMapGPGradientThread::phi(arr& y, arr& J, const ors::KinematicWorld& G, int t) {
  arr pos, JacPos;
  positionMap.phi(pos, JacPos, G);
  arr grad, hess;
  gp.writeAccess();
  gp().gradient(grad, pos);
  gp().hessianPos(hess, pos);
  gp.deAccess();

  arr Jac, ori;
  taskMap.phi(ori, Jac, G);

  y = ~ori*grad - ARR(length(ori)*length(grad));

  if(&J) {
    J = ~ori*hess*JacPos+~grad*Jac - length(grad)/length(ori)*~ori*Jac-length(ori)/length(grad)*~grad*hess*JacPos;
  }
}

TaskMapGPGradientThread::TaskMapGPGradientThread(Access_typed<GaussianProcess>& gp, const ors::KinematicWorld& world, const char* shapeName, ors::Vector vector)
  : gp(gp)
  , taskMap(vecTMT, world, shapeName, vector)
  , positionMap(posTMT, world, shapeName) {}





void TaskMap1DPosOrientation::phi(arr& y, arr& J, const ors::KinematicWorld& G, int t) {
  arr ori;
  orientationMap.phi(ori, NoArr, G);
  arr pos, JPos;
  positionMap.phi(pos, JPos, G);
  y = ~ori*pos;
  if(&J) {
    J = ~ori*JPos;
  }
}

TaskMap1DPosOrientation::TaskMap1DPosOrientation(const ors::KinematicWorld& world, const char* shapeName, const ors::Vector& vec)
  : orientationMap(vecTMT, world, shapeName, vec)
  , positionMap(posTMT, world, shapeName) {}


void TaskMapGP1D::phi(arr& y, arr& J, const ors::KinematicWorld& G, int t) {
  arr pos, JPos;
  positionMap.phi(pos, JPos, G);
  arr grad, H;
  gp.gradient(grad, pos);
  y = ~grad/length(grad)*pos;
  if(&J) {
    gp.hessianPos(H, pos);
    //J = ~grad*JPos;
    double l = length(grad);
    J = ~grad/l*JPos + ~pos*(H*JPos/l - grad*~grad*H*JPos/l/l/l);
  }
}

TaskMapGP1D::TaskMapGP1D(GaussianProcess& gp, const ors::KinematicWorld& world, const char* shapeName)
  : gp(gp)
  , positionMap(posTMT, world, shapeName) {}


void TaskMapGP1DThread::phi(arr& y, arr& J, const ors::KinematicWorld& G, int t) {
  arr pos, JPos;
  positionMap.phi(pos, JPos, G);
  arr grad, H;
  gp.set()->gradient(grad, pos);
  y = ~grad/length(grad)*pos;
  if(&J) {
    gp.set()->hessianPos(H, pos);
    //J = ~grad*JPos;
    double l = length(grad);
    J = ~grad/l*JPos + ~pos*(H*JPos/l - grad*~grad*H*JPos/l/l/l);
  }
}

TaskMapGP1DThread::TaskMapGP1DThread(Access_typed<GaussianProcess>& gp, const ors::KinematicWorld& world, const char* shapeName)
  : gp(gp)
  , positionMap(posTMT, world, shapeName) {}


void TaskMapGPVariance1DThread::phi(arr& y, arr& J, const ors::KinematicWorld& G, int t) {
  arr actPos, JPos;
  positionMap.phi(actPos, JPos, G);
  arr gradGP, gradV;
  gp.writeAccess();
  gp().gradient(gradGP, actPos);
  gp().gradientV(gradV, actPos);
  gp.deAccess();
  gradV.reshapeFlat();
  arr gradVT = gradV - (~gradGP*gradV).first()*gradGP/length(gradGP)/length(gradGP);
  gradVT = gradVT/length(gradVT);
  y = ARR(0.0);
  if(&J) {
    J = ~gradVT*JPos;
  }

}

TaskMapGPVariance1DThread::TaskMapGPVariance1DThread(Access_typed<GaussianProcess>& gp, const ors::KinematicWorld& world, const char* shapeName)
  : gp(gp)
  , positionMap(posTMT, world, shapeName) {}

#endif




