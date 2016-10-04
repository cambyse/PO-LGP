
#include <RL/Policy.h>
#include <RL/Environment.h>
#include <RL/Filter.h>
#include <RL/PolicySearch.h>
#include <RL/Optimization.h>
#include <Algo/kalman.h>

#include <Hardware/racer/racer.h>

struct RacerEnvironment : mdp::Environment, mdp::Filter{
  uint t;
  bool display;
  double exploration;
  double noise;
  double theta0;
  int fixedRandomSeed;

  Racer R;
  Kalman K;

  arr A,a,B; //store the transition linearizations
  arr C,c,W; //store the perception linearizations

  arr features;

  RacerEnvironment()
    : t(0), display(false){
    exploration = mlr::getParameter<double>("Racer/BalancingBenchmark/exploration", -1.);
    noise = mlr::getParameter<double>("Racer/BalancingBenchmark/noise", .1);
    theta0 = mlr::getParameter<double>("Racer/BalancingBenchmark/theta0", .2);
    fixedRandomSeed = mlr::getParameter<bool>("Racer/BalancingBenchmark/fixedRandomSeed", -1);
  }
  virtual ~RacerEnvironment() {}

  uint getStateDim(){ return 4; }
  uint getActionDim(){ return 1; }
  uint getFeatureDim(){ return 4; }

  void resetState(){
    t=0;
    R.q=0.;
    R.q_dot=0.;
  }

  bool transition(const arr& action, arr& perception, double& reward){
    //-- dynamics
    R.getDynamicsAB(A,a,B); //only to be stored for Kalman filter

    R.stepDynamics(action.scalar());
    if(display){
      R.gl().text.clear() <<t <<" ; " <<R.q(0) << " ; " <<R.q(1);
      R.gl().update();
    }

    //-- observations
    R.getObservation(perception, C,c,W);

    //-- costs
    double costs=0.;
    //deviation from (x,th)=(0,0)
    //    costs += .1*mlr::sqr(y(3)) + 1.*mlr::sqr(y(2)); // + 1.*sumOfSqr(w);
    costs += 1.*mlr::sqr(R.q(0)) + 10.*mlr::sqr(R.q(1)); // + 1.*sumOfSqr(w);

    //control costs
    costs += .1 * sumOfSqr(action);

    //big extra cost for crashing
    bool terminal=false;
    if(fabs(R.q(1))>30./180.*MLR_PI){ //greater 30 degrees
      costs += 1.*(t-t);
      terminal=true;
    }

    reward = -costs;
    return terminal;
  }

  void clearHistory(){
    K.initialize(cat(R.q, R.q_dot),1.*eye(4)); //correct initialization...
    features=K.b_mean;
  }

  void savePerception(const arr& action, const arr& perception){
    //-- state estimation
    K.stepPredict(eye(4)+R.tau*A, R.tau*(a+B*action),  diag(ARR(1e-6, 1e-6, 1., 1.)));
    K.stepObserve(perception, C, c, W);

    //-- construct features
    features=K.b_mean; //use the Kalman mean state as only feature
    features=cat(R.q, R.q_dot); //use the true state as feature
  }

  arr getObsEstimate(){ return features; }
};


struct LinearPolicy : mdp::Policy{
  double exploration;

  uint featureDim, actionDim;

  LinearPolicy(uint featureDim, uint actionDim)
    : featureDim(featureDim), actionDim(actionDim){
  }

  void sampleAction(arr& features, const arr& _theta, arr& action/*, arr& gradLog*/){
    arr theta=_theta;
    theta.reshape(actionDim, featureDim);
    action = theta * features;
    if(exploration>0.){
      double delta = exploration*rnd.gauss();
      action += delta;
//      gradLog += ~theta * delta / (exploration*exploration);
    }
  }

  void gradLogPol(arr& agentObservations, arr& theta, arr& actions, arr& gradLog)
  {
      //agentObservations is a matrix of observations! (of a whole rollout)
      arr thetaTemp = theta;
  //    thetaTemp.reshape(actionDim, agentObservations.d1);
      thetaTemp.reshape(actionDim, policyDim);
      gradLog.resizeAs(thetaTemp);
      gradLog.setZero();

      for(uint i=0; i<agentObservations.d0; i++)
      {
          for(uint j=0; j<actionDim; j++)
          {
              //arr aux = ~thetaTemp[j] * agentObservations[i];
              //aux = actions[i](j);
              gradLog[j] -= (~thetaTemp[j] * agentObservations[i] - actions(i,j)) * ~agentObservations[i] / (exploration*exploration);
          }
      }
  }

  uint getActionDim(){ return actionDim; }
  uint getPolicyDim(){ return featureDim; }


//  virtual void sampleAction(arr& currentAgentObs, const arr& theta, arr& action){ NIY }
//  virtual void gradLogPol(arr& agentObservations, arr& theta, arr& actions, arr& gradLog) { NIY }

};

int main(int argn, char** argv){

  RacerEnvironment R;
  LinearPolicy pi(R.getFeatureDim(), R.getActionDim());

  mdp::Optimization op;


  uint T = mlr::getParameter<uint>("Racer/BalancingBenchmark/T", 500);

  mdp::PolicySearch PS(R, pi, R, op, T, 10, 10, 1.);

  PS.updateREINFORCE();


}


