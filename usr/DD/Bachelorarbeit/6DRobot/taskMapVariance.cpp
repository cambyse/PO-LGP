#include "taskMapVariance.h"
#include <Motion/taskMap_default.h>

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

  y = ~ori*grad - ARR(length(ori)*length(grad));

  if(&J) {
    //J = Jac - hess*JacPos;
    J = ~ori*hess*JacPos+~grad*Jac - length(grad)/length(ori)*~ori*Jac-length(ori)/length(grad)*~grad*hess*JacPos;
  }
}

TaskMapGPGradient::TaskMapGPGradient(GaussianProcess& gp, const ors::KinematicWorld& world, const char* shapeName, ors::Vector vector)
  : gp(gp)
  , taskMap(vecTMT, world, shapeName, vector)
  , positionMap(posTMT, world, shapeName) {}
