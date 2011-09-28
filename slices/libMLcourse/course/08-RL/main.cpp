#include <MT/array.h>
#include <MT/opengl.h>

struct MDP{
  arr Ps;    //!< start distribution
  arr Psas;  //!< transition probs
  arr Ras;   //!< reward expectation as a function of (action,x_before)
  double gamma; //!< discounting factor
};

void loadMdpFile(MDP& mdp, const char* filename){
  ifstream fil;
  MT::open(fil,filename);

  mdp.Psas.readTagged(fil,"Psas");
  cout<<"read Psas"<<endl;
  
  arr gam;
  mdp.Ps.readTagged(fil,"Ps");
  cout<<"read Ps:"<<endl<<mdp.Ps<<endl;
  
  mdp.Ras.readTagged(fil,"Ras");
  cout<<"read Ras:"<<endl<<mdp.Ras<<endl;
  
  gam.readTagged(fil,"gamma");
  mdp.gamma = gam(0);
  cout<<"read gamma:"<<endl<<mdp.gamma<<endl;
  
//   cout<<"mdp.Psas(3,0,1)="<<mdp.Psas(3,0,1)<<endl;
}

void simulate(uint& state_new, double& reward, uint action, uint state_old,const MDP& mdp){
  uint n=mdp.Ps.N;

  reward = mdp.Ras(action,state_old);

  arr p(n);
  for(uint s=0;s<n;s++) p(s) = mdp.Psas(s,action,state_old); //same as Psas(:,action,state) in matlab
//   cout<<"action="<<action<<"  state_old="<<state_old<<"   p="<<p<<endl;
  state_new = SUS(p); //sample from p
}

#define Q_LEARNING__GREEDY 1
#define E_CUBE 2
#define TIMESTEPS 1000000

int main(int argn, char **argv){
  uint rl_algorithm = 1;
  
  rnd.seed(1000);
  
  MT::initCmdLine(argn,argv);

  //display
  OpenGL gl;
  
  //load the file
  MDP mdp;
  loadMdpFile(mdp, "mdp_e12.mdp");
  //cout <<mdp.Psas <<mdp.Ps <<mdp.Ras <<mdp.gamma <<endl;
    
  //initialize with a random walker
  uint S=mdp.Ps.N,A=mdp.Psas.d1;
  arr pi(S,A); //store the policy such that pi[s] is the distribution over actions in s
  pi = 1./A;   //random policy pi(a|s) = 1/A
  arr Q(S,A);  //same with Q function
  Q.setZero(); //initialize Q(s,a)=0

  //learning parameters
  double epsilon_start = .3;
  double epsilon_coeff = 0.001;
//   double epsilon_coeff = 0.01;
  
  double alpha = .1;
  
  uint state_start = argmax(mdp.Ps);
  uint state_old, state_new, action;
  double reward;
  
  
  uintA c_p_SA(S,A);
  c_p_SA.setZero();
  uintA c_p_SAS(S,A,S);
  c_p_SAS.setZero();
  arr c_r_SA(S,A);
  c_r_SA.setZero();
  uint c_random_actions=0;
  double e3_threshold = 5.0;
  
  arr estimate_p_SAS(S,A,S);
  arr estimate_r_AS(A,S);

//   rnd.clockSeed();

  state_new = SUS(mdp.Ps); //sample from the start distribution
  ofstream fil("z.rewards");
  
  uint s, a, s_suc;
  
  for(uint t=0;t<TIMESTEPS;t++){
    state_old = state_new;
    
    bool explore_action = false;
    
    // Policy / Action choice
    if (rl_algorithm == Q_LEARNING__GREEDY) {
      double epsilon = epsilon_start / (1+t*epsilon_coeff);
//       cout<<"epsilon="<<epsilon<<endl;
      if(rnd.uni()<epsilon){ //with probability epsilon: choose random action
        action=rnd(A);
        c_random_actions++;
        explore_action = true;
      }else{
        // greedy policy
        pi.setZero();
        for(uint s=0;s<S;s++){
//           uint max_action = argmax(Q[s] + 1e-10*randn(A,1));
          uint max_action = argmax(Q[s]);
          pi(s,max_action) = 1.;
        }
        action = SUS(pi[state_old]); //sample from pi(:|s)
      }
    }
    else if (rl_algorithm == E_CUBE) {
      boolA known_states(S);
      known_states.setUni(false);
      FOR1D(known_states, s) {
        if (c_p_SA(s,0) >= e3_threshold  &&  c_p_SA(s,1) >= e3_threshold) {
          known_states(s) = true;
        }
      }
      // # if known: plan
      if (known_states(state_old)) {
        MDP mdp_known;
        mdp_known.Ps(S);
        mdp_known.Ps.setZero();
        mdp_known.Ps(state_old) = 1.0;
        
        mdp_known.Psas = estimate_p_SAS;
        mdp_known.Ras = estimate_r_AS;
        
        // TODO plan
        
        // falls wert von state_old < threshold
        // unknown mdp
        MDP mdp_unknown;
        mdp_unknown.Ps(S);
        mdp_unknown.Ps.setZero();
        mdp_unknown.Ps(state_old) = 1.0;
        
        mdp_unknown.Psas = estimate_p_SAS;
        FOR1D(known_states, s) {
          if (!known_states(s)) {
            for (s_suc=0; s_suc<S; s_suc++) {
              if (s_suc == s) {
                mdp_unknown.Psas(s,0,s_suc) = 1.;
                mdp_unknown.Psas(s,1,s_suc) = 1.;
              }
              else {
                mdp_unknown.Psas(s,0,s_suc) = 0.;
                mdp_unknown.Psas(s,1,s_suc) = 0.;
              }
            }
          }
        }
        
        mdp_unknown.Ras(S,A);
        mdp_unknown.Ras.setZero();
        FOR1D(known_states, s) {
          if (!known_states(s)) {
            mdp_unknown.Ras(0,s) = 20.;
            mdp_unknown.Ras(1,s) = 20.;
          }
        }
        
        // TODO da drin planen
        
      }
      else {
        if (c_p_SA(s,0) < c_p_SA(s,1))
          action = 0;
        else
          action = 1;
      }
    }
    else
      NIY;

    simulate(state_new, reward, action, state_old, mdp);
    c_p_SA(state_old, action)++;
    c_p_SAS(state_old, action, state_new)++;
    c_r_SA(state_old, action) += reward;
    
//     cout <<"s=" <<state_old  <<" a=" <<action  <<" s'=" <<state_new  <<" r=" <<reward  <<endl;
    if (TIMESTEPS > 100000) {
      if (t % 5 == 0)
        fil <<t <<' ' <<reward <<' ' <<max(Q[state_start]) << " " << explore_action<<endl;
    }
    else if (TIMESTEPS > 20000) {
      if (t % 5 == 0)
        fil <<t <<' ' <<reward <<' ' <<max(Q[state_start]) << " " << explore_action<<endl;
    }
    
    
    // Updates
    if (rl_algorithm == Q_LEARNING__GREEDY) {
      //update the Q(s,a) function
      double maxQ = max(Q[state_new]);
      Q(state_old,action) += alpha * (reward + mdp.gamma * maxQ - Q(state_old,action));
//       cout<<"max[Q(state_new="<<state_new<<")] = "<<max(Q[state_new])<<endl;
//       cout<<"Q(state_old="<<state_old<<",action="<<action<<") = "<<Q(state_old,action)<<endl;
    }
    else if (rl_algorithm == E_CUBE) {
      FOR2D(c_p_SA, s, a) {
        for (s_suc=0; s_suc<S; s_suc++) {
          if (c_p_SA(s,a) > 0.)
            estimate_p_SAS(s_suc,a,s) = (1.0 * c_p_SAS(s,a,s_suc)) / c_p_SA(s,a);
          else {
            if (s == s_suc)
              estimate_p_SAS(s_suc,a,s) = 1.;
            else
              estimate_p_SAS(s_suc,a,s) = 0.;
          }
        }
      }
      
      FOR2D(estimate_r_AS, s, a) {
        estimate_r_AS(s,a) = (1.0 * c_r_SA(s,a)) / c_p_SA(s,a);
      }
    }
    
    
    //display
    if(false){
      byteA img(3,3);
      img=255;
      img.elem(state_new) = 128;
      img.elem(7)=img.elem(4) = 0;
      gl.watchImage(img,false,10);
      MT::wait(.1);
    }
  }
  
  FOR2D(c_p_SA, s, a) {
    for (s_suc=0; s_suc<S; s_suc++) {
      if (c_p_SA(s,a) > 0.)
        estimate_p_SAS(s,a,s_suc) = (1.0 * c_p_SAS(s,a,s_suc)) / c_p_SA(s,a);
      else
        estimate_p_SAS(s,a,s_suc) = 0.;
    }
  }
  
  cout<<"Q:"<<endl<<Q<<endl<<endl;
  cout<<"c_p_SA:"<<endl<<c_p_SA<<endl<<endl;
  cout<<"c_p_SAS:"<<endl<<c_p_SAS<<endl<<endl;
  cout<<"c_random_actions = "<<c_random_actions<<" ("<<((1.0 * c_random_actions) / TIMESTEPS)<<"%)"<<endl<<endl;
  cout<<"estimate_p_SAS:"<<endl<<estimate_p_SAS<<endl<<endl;
  
  
  fil.close();
  gnuplot("plot 'z.rewards' us 1:2 title 'rewards', 'z.rewards' us 1:3 title 'Value(start)', 'z.rewards' us 1:4 title 'explore actions'",NULL,true);

  return 0;
}
