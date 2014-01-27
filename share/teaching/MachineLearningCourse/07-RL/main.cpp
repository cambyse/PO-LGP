#include <Core/array.h>
#include <Gui/opengl.h>

struct MDP{
  arr Ps;    ///< start distribution
  arr Psas;  ///< transition probs
  arr Ras;   ///< reward expectation as a function of (action,x_before)
  double gamma; ///< discounting factor
};

void loadMdpFile(MDP& mdp, const char* filename){
  ifstream fil;
  MT::open(fil,filename);
  arr gam;
  mdp.Psas.readTagged(fil,"Psas");
  mdp.Ps.readTagged(fil,"Ps");
  mdp.Ras.readTagged(fil,"Ras");
  gam.readTagged(fil,"gamma");
  mdp.gamma = gam(0);
  
  for (uint i=0; i<5; i++) {
    cout<<"mdp.Psas(0,"<<i<<",8) = "<<mdp.Psas(0,i,8)<<endl;
    cout<<"mdp.Psas(1,"<<i<<",8) = "<<mdp.Psas(1,i,8)<<endl;
    cout<<"mdp.Psas(2,"<<i<<",8) = "<<mdp.Psas(2,i,8)<<endl;
    cout<<"mdp.Psas(3,"<<i<<",8) = "<<mdp.Psas(3,i,8)<<endl;
    cout<<"mdp.Psas(4,"<<i<<",8) = "<<mdp.Psas(4,i,8)<<endl;
    cout<<"mdp.Psas(5,"<<i<<",8) = "<<mdp.Psas(5,i,8)<<endl;
    cout<<"mdp.Psas(6,"<<i<<",8) = "<<mdp.Psas(6,i,8)<<endl;
    cout<<"mdp.Psas(7,"<<i<<",8) = "<<mdp.Psas(7,i,8)<<endl;
    cout<<"mdp.Psas(8,"<<i<<",8) = "<<mdp.Psas(8,i,8)<<endl;
    cout<<endl;
  }
}

void simulate(uint& state_new, double& reward, uint action, uint state_old,const MDP& mdp){
  uint n=mdp.Ps.N;

  reward = mdp.Ras(action,state_old);

  arr p(n);
  for(uint s=0;s<n;s++) p(s) = mdp.Psas(s,action,state_old); //same as Psas(:,action,state) in matlab
  state_new = SUS(p); //sample from p
}

int main(int argc, char **argv){
  MT::initCmdLine(argc,argv);

  //display
  OpenGL gl;
  
  //load the file
  MDP mdp;
  loadMdpFile(mdp, "miniMaze.mdp");
  //cout <<mdp.Psas <<mdp.Ps <<mdp.Ras <<mdp.gamma <<endl;
    
  //initialize with a random walker
  uint S=mdp.Ps.N,A=mdp.Psas.d1;
  arr pi(S,A); //store the policy such that pi[s] is the distribution over actions in s
  pi = 1./A;   //random policy pi(a|s) = 1/A
  arr Q(S,A);  //same with Q function
  Q.setZero(); //initialize Q(s,a)=0

  //learning parameters
  double epsilon = .1;
  double alpha = .1;
  
  uint state_start = argmax(mdp.Ps);
  uint state_old, state_new, action;
  double reward;

  rnd.clockSeed();

  state_new = SUS(mdp.Ps); //sample from the start distribution
  ofstream fil("z.rewards");
  for(uint t=0;t<10000;t++){
    state_old = state_new;

    if(rnd.uni()<epsilon){ //with probability epsilon: choose random action
      action=rnd(A);
      //in Q(lambda): set all eligibility traces zero!
    }else{
      action = SUS(pi[state_old]); //sample from pi(:|s)
    }

    simulate(state_new, reward, action, state_old, mdp);

//     cout <<"s=" <<state_old  <<" a=" <<action  <<" s'=" <<state_new  <<" r=" <<reward  <<endl;
    fil <<t <<' ' <<reward <<' ' <<max(Q[state_start]) <<endl;
    
    //update the Q(s,a) function
    double maxQ = max(Q[state_new]);
    Q(state_old,action) += alpha * (reward + mdp.gamma * maxQ - Q(state_old,action));
    
    //update the policy to be greedy policy (exploration handled during action selection)
    pi.setZero();
    for(uint s=0;s<S;s++){
      uint max_action = argmax(Q[s] + 1e-10*randn(A,1));
      pi(s,max_action) = 1.;
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
  fil.close();
  gnuplot("plot 'z.rewards' us 1:2 title 'rewards', 'z.rewards' us 1:3 title 'Value(start)'",NULL,true);

  return 0;
}
