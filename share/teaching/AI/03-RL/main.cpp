#include <Core/array.h>
#include <Gui/opengl.h>

struct MDP{
  arr Ps;    ///< start distribution
  arr Psas;  ///< transition probs
  arr Ras;   ///< reward expectation as a function of (action,x_before)
  double gamma; ///< discounting factor
};

MDP circleWorld(){
  ifstream fil;
  mlr::open(fil,"circleWorld.txt");
  MDP mdp;
  fil >>mdp.Psas;
  fil >>mdp.Ps;
  fil >>mdp.Ras;
  fil >>mdp.gamma;

  checkNormalization(mdp.Psas);
  checkNormalization(mdp.Ps);
  //  cout <<mdp.Psas <<endl <<mdp.Ps <<endl <<mdp.Ras <<endl <<mdp.gamma <<endl;

  return mdp;
}

arr valueIteration(const MDP& mdp, uint K=10){
  uint S=mdp.Ps.N, A=mdp.Psas.d1;
  arr V(S), Q(S,A);
  V.setZero();
  for(uint k=0;k<K;k++){
    cout <<"k=" <<k <<" V= " <<V <<endl;
    //compute the Q-function
    for(uint s=0; s<S; s++) for(uint a=0; a<A; a++){
      Q(s,a) = mdp.Ras(a,s);
      for(uint ss=0; ss<S; ss++) Q(s,a) += mdp.gamma * mdp.Psas(ss, a, s) * V(ss);
    }
    //compute the V-function
    for(uint s=0; s<S; s++) V(s) = max(Q[s]);
  }
  return V;
}


void simulate(uint& state_new, double& reward, uint action, uint state_old,const MDP& mdp){
  uint n=mdp.Ps.N;

  reward = mdp.Ras(action,state_old);

  arr p(n);
  for(uint s=0;s<n;s++) p(s) = mdp.Psas(s,action,state_old); //same as Psas(:,action,state) in matlab
  state_new = sampleMultinomial(p); //sample from p
}

void Qlearning(const MDP& mdp){
  //initialize with a random walker
  uint S=mdp.Ps.N,A=mdp.Psas.d1;

  arr pi(S,A); //store the policy such that pi[s] is the distribution over actions in s
  pi = 1./A;   //random policy pi(a|s) = 1/A

  arr Q(S,A);  //same with Q function
  Q.setZero(); //initialize Q(s,a)=0

  //learning parameters
  double epsilon = 1.;
  double alpha = .01;

  uint state_start = argmax(mdp.Ps);
  uint state_old, state_new, action;
  double reward;

  rnd.clockSeed();

  state_new = sampleMultinomial(mdp.Ps); //sample from the start distribution
  ofstream fil("z.rewards");
  for(uint t=0;t<100000;t++){
    state_old = state_new;

    if(rnd.uni()<epsilon){ //with probability epsilon: choose random action
      action=rnd(A);
      //in Q(lambda): set all eligibility traces zero!
    }else{
      action = sampleMultinomial(pi[state_old]); //sample from pi(:|s)
    }

    simulate(state_new, reward, action, state_old, mdp);

    cout <<"s=" <<state_old  <<" a=" <<action  <<" s'=" <<state_new  <<" r=" <<reward  <<endl;
    fil <<t <<' ' <<reward <<' ' <<max(Q[state_start]) <<endl;

    // update the Q(s,a) function
    double maxQ = max(Q[state_new]);
    Q(state_old,action) += alpha * (reward + mdp.gamma * maxQ - Q(state_old,action));

    // update the policy to be greedy policy (exploration handled during action selection)
    pi.setZero();
    for(uint s=0;s<S;s++){
      uint max_action = argmax(Q[s] + 1e-10*randn(A,1));
      pi(s,max_action) = 1.;
    }

  }
  fil.close();
  gnuplot("plot 'z.rewards' us 1:2 title 'rewards', 'z.rewards' us 1:3 title 'Value(start)'",NULL,true);

  //compute the V-function
  arr V(S);
  for(uint s=0; s<S; s++) V(s) = max(Q[s]);
  cout <<"V=" <<V <<endl;

}

int main(int argc, char **argv){
  mlr::initCmdLine(argc,argv);

  //load the MDP
  MDP mdp = circleWorld();
    
  valueIteration(mdp);
  //  Qlearning(mdp);

  return 0;
}
