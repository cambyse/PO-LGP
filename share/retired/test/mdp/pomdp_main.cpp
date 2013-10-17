#ifdef MT_MSVC
#  define MT_IMPLEMENTATION
#endif
#include <vector>
#include <iostream>
#include <string>
#include <map>

//#include <Gui/plot.h>
#include <MT/infer.h>
#include <MT/mdp_help.h>

#include "pomdp.h"
#include "mdpInterface.h"

void glDisplayGrey   (const arr &x,uint d0,uint d1,bool wait,uint win){ HALT("only with opengl"); }
void glDisplayRedBlue(const arr &x,uint d0,uint d1,bool wait,uint win){ HALT("only with opengl"); }

struct globalParameters{
  MT::String problem;
  uint d0,d1,seed;
  uint Tcut,EMiters;
  uint valueCheckInterval;
  uint OutputFSC;
  uint FixPolicy;
  uint EscapeLocalMin;
  uint SetAMPLPolicy;
  bool maxMstep;
  bool adaptP0;
} PARAMS;

void makeNodeActionDeterministic(FSC_lev2& fsc, arr& V_01_x, bool hierarchy) { 

  uint d0=fsc.P01y0.d0,d1=fsc.P1y01.d0,da=fsc.Pa0.d0,dy=fsc.P1y01.d1;
  const uint d0Fixed=fsc.P01y0.d0;
  // Find, one by one, nodes that are generating actions in a stochastic fashion
  for(uint j=0;j<d0Fixed;++j){ // j n0's
    double cumProb=0.0; 
    for(uint i=0;i<da;++i) { // i actions
      cumProb += fsc.Pa0(i,j);
      uint d0New=d0;
      if(fsc.Pa0(i,j) > 0.01 && fsc.Pa0(i,j) < 0.99) {
      	cout << " ADDING\t\t" << std::flush;
        uint oldNode=j, newNode=d0, action=i;
        if(cumProb < 0.99) { 
          d0New=d0+1;
          //1- Add a node
          fsc.Pa0.tensorAddRange(1);
          fsc.P01y0.tensorAddRange(0);fsc.P01y0.tensorAddRange(3);
          fsc.P1y01.tensorAddRange(2);
          fsc.P0.tensorAddRange(0);
          arr v_01_x((d0+1)*d1,V_01_x.d1);

          // New Node does action deterministically
          fsc.Pa0(action,newNode)=1.;
          V_01_x.resizeAs(v_01_x);
          if(!hierarchy) { 
            // similarly for P01y0
            for(uint ii=0;ii<d0New;++ii) for(uint jj=0;jj<d1;++jj){
              for(uint k=0;k<dy;++k) for(uint l=0;l<d0New;++l) {
                if(ii==d0) // here scale by the previous prob of doing the action.
                  fsc.P01y0(TUP(ii,jj,k,l)) = fsc.P01y0(TUP(oldNode,jj,k,l))*fsc.Pa0(i,j);
                else if(l==d0)
                  fsc.P01y0(TUP(ii,jj,k,l)) = fsc.P01y0(TUP(ii,jj,k,oldNode));
              }
            }
            // similarly for P10y1
            for(uint ii=0;ii<d1;++ii) for(uint jj=0;jj<dy;++jj){
              for(uint l=0;l<d1;++l) {
                fsc.P1y01(TUP(ii,jj,d0,l)) = fsc.P1y01(TUP(ii,jj,oldNode,l));
              }
            }
          }else{
            // Stretch hierarchical params
            fsc.PE_01.tensorAddRange(0);
            fsc.P_E_0y0.tensorAddRange(0);fsc.P_E_0y0.tensorAddRange(2);
            fsc.Pe0.tensorAddRange(1); // TODO: don't assume hierarchical, check
            for(uint ii=0;ii<fsc.Pe0.d0;++ii) fsc.Pe0(ii, d0) = fsc.Pe0(ii, oldNode);

            for(uint ii=0;ii<fsc.PE_01.d0;++ii) for(uint jj=0;jj<fsc.PE_01.d1;++jj) {
              if(ii==d0) // here scale by the previous prob of getting to that node
                fsc.PE_01(ii,jj) = fsc.PE_01(oldNode,jj)*fsc.Pa0(i,j);
            }

            for(uint ii=0;ii<d0New;++ii) for(uint jj=0;jj<dy;++jj){
              for(uint k=0;k<d0New;++k) { 
                if(ii==d0) // here scale by the previous prob of doing the action.
                  fsc.P_E_0y0(TUP(ii,jj,k)) = fsc.P_E_0y0(TUP(oldNode,jj,k))*fsc.Pa0(i,j);
                else if(k==d0)
                  fsc.P_E_0y0(TUP(ii,jj,k)) = fsc.P_E_0y0(TUP(ii,jj,oldNode));
              }
            }
            hierarchicalToFactors(fsc);
          } 
        }else{ // cumProb > 0.99
          if(!hierarchy) { 
          // First fix old transition to/from the old node
            for(uint jj=0;jj<d1;++jj){
              for(uint k=0;k<dy;++k) for(uint l=0;l<d0New;++l) {
                fsc.P01y0(TUP(oldNode,jj,k,l)) *= fsc.Pa0(action,oldNode);
              }
            }
          }else{
            for(uint jj=0;jj<dy;++jj) for(uint k=0;k<d0New;++k) 
                fsc.P_E_0y0(TUP(oldNode,jj,k)) *= fsc.Pa0(action,oldNode);

            for(uint jj=0;jj<fsc.PE_01.d1;++jj)
              fsc.PE_01(oldNode,jj) *= fsc.Pa0(action,oldNode);

            cout << "P0 normalized, " << std::endl; fsc.P0.tensorCheckCondNormalization(1);
            cout << "Pe0 normalized, " << std::endl; fsc.Pe0.tensorCheckCondNormalization(1);
            cout << "PE_01 normalized, " << std::endl; fsc.PE_01.tensorCheckCondNormalization(1,1e-2);
            fsc.PE_01.tensorCondNormalize(1);
            cout << "P_E_0y0 normalized. " << std::endl; fsc.P_E_0y0.tensorCheckCondNormalization(1,1e-2);
            fsc.P_E_0y0.tensorCondNormalize(1);
            
            hierarchicalToFactors(fsc);
          }
          
          // Fix old node Action
          for(uint ii=0;ii<da;++ii) fsc.Pa0(ii,oldNode)=0.;
          fsc.Pa0(action,oldNode)=1.;

          // Re-distribute P0
          uint numNewActions = d0New-fsc.P0.d0; 
          fsc.P0(oldNode) /= (numNewActions+1);
          for(uint ii=0;ii<numNewActions;++ii){
            fsc.P0(d0Fixed+ii)=fsc.P0(oldNode);
          }

          cout << "Added " << numNewActions+1 <<  " Nodes: " << std::endl;
          fsc.Pa0.tensorCheckCondNormalization(1);
          cout << "P1y01 normalized, "<< std::endl; fsc.P1y01.tensorCheckCondNormalization(1);
          cout << "P01y0 normalized, "<< std::endl; fsc.P01y0.tensorCondNormalize(1);
          cout << "P01y0 normalized, " << std::endl; fsc.P01y0.tensorCheckCondNormalization(1,5e-3);
          fsc.P01y0.tensorCondNormalize(1);
          cout << "Done\n";
        }
      }
      d0=d0New;
    }
  }
} 

// Tries to detect when the algorithm has reached a local optimum by looking at
// the last 10 values of the objective.
void detectLocalOpt(double Like, std::list<double>& LikeHistory, FSC_lev2& fsc, 
                    uint exits, arr& V_01_x, bool hierarchy) { 

  double tol=1e-3;
  LikeHistory.push_front(Like);
  if(LikeHistory.size() >= 10) {
    LikeHistory.pop_back();
    if(fabs(LikeHistory.front()-LikeHistory.back())/fabs(LikeHistory.front()) <tol )  {
      cout << LikeHistory.front() << " " << LikeHistory.back() << " " << fabs(LikeHistory.front()-LikeHistory.back())/fabs(LikeHistory.front())<< endl;
      // call some heuristic to get out of it
      makeNodeActionDeterministic(fsc,V_01_x,hierarchy);
      //initHierarchical(exits,fsc);
      LikeHistory.clear();
    }
  }
}

void fixPolicyH(FSC_lev2 &fsc) { 
  
  cout << "Fixing H-policy\n";
  // Fix actions into nodes
  // (i,j), where i is the action and j the node
  fsc.Pa0 = 0.0; 
  fsc.Pa0(0,0)=1.0;fsc.Pa0(1,1)=1.0;fsc.Pa0(2,2)=1.0;fsc.Pa0(3,3)=1.0;
  //fsc.Pa0.tensorCondNormalize(1);

  fsc.P1=0.;fsc.P1(0)=1.; 
  fsc.P0=0.;fsc.P0(0)=1.;
  // P01y0
  fsc.P01y0 = 0.0;
  // nodes 2&3 are exit nodes
//  fsc.P01y0(TUP(1,0,0,0))=1.;fsc.P01y0(TUP(2,0,0,1))=1.;
//  fsc.P01y0(TUP(1,1,0,0))=1.;fsc.P01y0(TUP(2,1,0,1))=1.;
//  fsc.P01y0(TUP(1,2,0,0))=1.;fsc.P01y0(TUP(2,2,0,1))=1.;
//  fsc.P01y0(TUP(1,3,0,0))=1.;fsc.P01y0(TUP(2,3,0,1))=1.;
//  fsc.P01y0(TUP(0,0,0,2))=1.;fsc.P01y0(TUP(0,1,0,2))=1.;fsc.P01y0(TUP(0,2,0,2))=1.;fsc.P01y0(TUP(3,3,0,2))=1.;
//  fsc.P01y0(TUP(0,0,0,3))=1.;fsc.P01y0(TUP(0,1,0,3))=1.;fsc.P01y0(TUP(0,2,0,3))=1.;//fsc.P01y0(TUP(0,3,0,3))=1.;
//  //fsc.P01y0.tensorCondNormalize(1);
//  // P1y01
//  fsc.P1y01 = 0.0;
//  fsc.P1y01(TUP(1,0,0,0))=1.;fsc.P1y01(TUP(2,0,0,1))=1.0;fsc.P1y01(TUP(3,0,0,2))=1.;
//  fsc.P1y01(TUP(0,0,0,3))=1.;
////  fsc.P1y01(TUP(0,0,0,0))=1.;fsc.P1y01(TUP(0,0,1,0))=1.;fsc.P1y01(TUP(1,0,2,0))=1.;
////  fsc.P1y01(TUP(1,0,0,1))=1.;fsc.P1y01(TUP(1,0,1,1))=1.;fsc.P1y01(TUP(2,0,2,1))=1.;
////  fsc.P1y01(TUP(2,0,0,2))=1.;fsc.P1y01(TUP(2,0,1,2))=1.;fsc.P1y01(TUP(3,0,2,2))=1.;
////   fsc.P1y01(TUP(0,0,0,3))=1.;fsc.P1y01(TUP(0,0,1,3))=1.;fsc.P1y01(TUP(0,0,2,3))=1.;fsc.P1y01(TUP(0,0,3,3))=1.;
//  //fsc.P1y01.tensorCondNormalize(1);
  ///

  // fix exit states
  fsc.Pe0 = 0.;
  fsc.Pe0(0,0)=1.;fsc.Pe0(0,1)=1.;fsc.Pe0(0,2)=0.;fsc.Pe0(0,3)=0.;// fsc.Pe0(0,4)=0.;fsc.Pe0(0,5)=0.;fsc.Pe0(0,6)=0.;
  fsc.Pe0(1,0)=0.;fsc.Pe0(1,1)=0.;fsc.Pe0(1,2)=1.;fsc.Pe0(1,3)=1.;// fsc.Pe0(1,4)=1.;fsc.Pe0(1,5)=1.;fsc.Pe0(1,6)=1.;

  // fix base-level
  // P_E_0y0(i,j,k), i=n0', j=y, k=n0
  fsc.P_E_0y0=0.0;
  fsc.P_E_0y0(1,0,0)=1.;fsc.P_E_0y0(2,0,1)=1.;
  fsc.P_E_0y0(2,0,2)=1.;fsc.P_E_0y0(3,0,3)=1.;
  // fix vertical transitions
  // PE_01
  fsc.PE_01=0.0;
  fsc.PE_01(0,0)=1.0;fsc.PE_01(0,1)=1.0;fsc.PE_01(0,2)=1.0;fsc.PE_01(3,3)=1.0;
  // fix upper-level
  // PE_1y1 and P_E_11
  fsc.PE_1y1=0.0;
  fsc.PE_1y1(1,0,0)=1.0;fsc.PE_1y1(2,0,1)=1.0;fsc.PE_1y1(3,0,2)=1.0;fsc.PE_1y1(0,0,3)=1.0;
  fsc.P_E_11=0.0;
  fsc.P_E_11(0,0)=1.0;fsc.P_E_11(1,1)=1.0;fsc.P_E_11(2,2)=1.0;fsc.P_E_11(3,3)=1.0;

  hierarchicalToFactors(fsc);
  //makeHierarchical(2, fsc.P01y0, fsc.P1y01);
}

// This fixes policy for the chainofChains problem
void fixPolicy2(FSC_lev2 &fsc) { 

  cout << fsc.P01y0.d0 << " " << fsc.P01y0.d1 << endl;
  cout << "fixing policy\n";

  fsc.P0 = 0.0; fsc.P0(0)=1.0;
  
  // Fix actions into nodes
  // (i,j), where i is the action and j the node
  fsc.Pa0 = 0.0; 
  fsc.Pa0(0,0)=1.0;fsc.Pa0(1,1)=1.0;fsc.Pa0(2,2)=1.0;
  fsc.Pa0(0,3)=1.0;fsc.Pa0(1,4)=1.0;fsc.Pa0(2,5)=1.0;
  fsc.Pa0(0,6)=1.0;fsc.Pa0(1,7)=1.0;fsc.Pa0(2,8)=1.0;
  fsc.Pa0(3,9)=1.0;

  // Fix transitions
  // (i,j,k,l), i=n0', j=n1, k=y, l=n0
  fsc.P01y0 = 0.0;
  fsc.P01y0(TUP(1,0,0,0))=1.0;fsc.P01y0(TUP(2,0,0,1))=1.0;fsc.P01y0(TUP(3,0,0,2))=1.0;
  fsc.P01y0(TUP(4,0,0,3))=1.0;fsc.P01y0(TUP(5,0,0,4))=1.0;fsc.P01y0(TUP(6,0,0,5))=1.0;
  fsc.P01y0(TUP(7,0,0,6))=1.0;fsc.P01y0(TUP(8,0,0,7))=1.0;fsc.P01y0(TUP(9,0,0,8))=1.0;
  fsc.P01y0(TUP(0,0,0,9))=1.0;

}


// Calculate the value function given a fixed policy. 
// This does a full Bellman-backup.
void V(arr& V0x, const MDP& mdp, const arr& P0ay0, const arr& Pa0, const arr& P0y0) {
  
  uint d0=P0y0.d2, da=mdp.Pxax.d1, dy=mdp.Pyxa.d0, dx=mdp.Pxax.d0;

  // all of the future rewards
  arr Pay0x;
  Pay0x.resize(TUP(da, dy, d0, dx));
  tensor(Pay0x , P0ay0, TUP(4,0,1,2), V0x, TUP(4,3), 1);
  arr Pa0x_(da, d0, dx);
  tensor(Pa0x_ , Pay0x, TUP(0,3,1,2), mdp.Pyxa, TUP(3,2,0), 1); 
  arr Pa0x(da, d0, dx);
  tensor(Pa0x, Pa0x_, TUP(0,1,3), mdp.Pxax, TUP(3,0,2), 1);
  Pa0x *= mdp.gamma;
  
  // the immediate reward
  // Pa0xB is only a 3D tensor although its name indicates otherwise
  arr Pa0xB(da, d0, dx);
  tensor(Pa0xB, Pa0, TUP(0,1), mdp.Rax, TUP(0,2), 0);
  
  // put everything together
  //arr V0x_(V0x.d0, V0x.d1);
  Pa0x += Pa0xB; 
  //tensor(V0x, Pa0xB, TUP(2,0,1), Pa0x, TUP(2,1,0), 1);
  //cout << "tensoring 3\n";
  // sum out a
  //arr P0x(P0y0.d2, Pxax.d2);
  eliminatePartial(V0x, Pa0x, 0);
  
  // V0x should contain the completely updated value table
  // this is one back-up, we probably want to back-up as long as we don't reach
  // some kind of an equilibrium.
}

// Evaluate the policy by finding the fixed point of the Bellman equation (with
// the policy constraints). This is very straight-forward, you calculate the
// value function (V0x) and iterate until convergence (with a fixed policy).
// Note: although only this function deals with V0x, it's a good idea to cache
// it in order to converge with less iterations
// Note: Watchout, Rxax is actually 2d (i.e. Rax = Rxax)
double EvalPolicy(arr& V0x, const MDP& mdp, const FSC_lev1& fsc){ 
  
  uint d0=fsc.P0y0.d2, da=mdp.Pxax.d1, dy=mdp.Pyxa.d0, dx=mdp.Pxax.d0;
  
  // Make P0ay0 from Pa0 and P0y0
  arr P0ay0;
  P0ay0.resize(TUP(d0, da, dy, d0));
  tensor(P0ay0, fsc.P0y0, TUP(0,2,3), fsc.Pa0, TUP(1,3), 0);	
  
  double ExpectedValue = 0.0;
  double tol= 1e-4; // it looks, from small tests, that this value could be as 
  // high as 1e-2
  uint i=0;
  while(1) {
    // calculate V
    V(V0x, mdp, P0ay0, fsc.Pa0, fsc.P0y0);
    double IteratedExpectedValue = 0.0;
    // by default we'll use q_0 = 0 (i.e., the start node is node 0).
    for(uint j=0;j<dx;++j) IteratedExpectedValue += V0x(0,j)*mdp.Px(j);
    // have we converged yet?
    // the i>3 is a hack but it happens that V0x(0,i) doesn't change but
    // it's other values do ...  maybe we should look at V0x in its entirety
    // instead of only at one value ... Or look at the past two iterations.
    if(fabs(ExpectedValue-IteratedExpectedValue) < tol && i>20)  break; 
    //if(fabs(ExpectedValue-IteratedExpectedValue) < tol)  break;
    else ExpectedValue = IteratedExpectedValue;
    ++i;
  }
  cout << "Exp(Value) = " << ExpectedValue << endl;
  return ExpectedValue;
}

void convertProblem(char* problem){
  MDP mdp;
  
  readMDP(problem, mdp.Pxax, mdp.Pyxa, mdp.Px, mdp.Rax, mdp.gamma);
  savePOMDP(STRING(problem<<".arr"),mdp);
}

//================================================================================
//
// simple maze test problem
//
#if 0
void optimalPolicyForPaint(FSC_lev1 &fsc){
  fsc.Pa0.setId();
  fsc.P0y0 <<"[ \
		  0 1 0 0 \
		  0 0 0 0 \
		  \
		  0 0 1 1 \
		  0 0 1 1 \
		  \
		  1 0 0 0 \
		  1 0 0 0 \
		  \
		  0 0 0 0 \
		  0 1 0 0 \
		  ]";
  fsc.Pa0.tensorCheckCondNormalization(1);
  fsc.P0y0.reshape(d0,dy,d0);
  fsc.P0y0.tensorCheckCondNormalization(1);
#if 1 //make a bit noisy
  rndUni(fsc.P0y0,.0,.9,true);
  rndUni(fsc.Pa0, .0,.9,true);
  fsc.P0y0.tensorCondNormalize(1);
  fsc.Pa0.tensorCondNormalize(1);
#endif	
}
#endif

void go_lev1(const char *problem){
  //load the problem
  MDP mdp;
  if(!strcmp(problem,"simpleMaze")){
    generateProblem(simpleMaze,mdp); //tinyMaze //heavenAndHell
    mdp.gamma=.9;
  }else{
    cout <<"reading POMDP file: " <<problem <<endl;
    loadPOMDP(problem, mdp);
    mazeD0=4; mazeD1=4;
  }
  
  //init policy
  PARAMS.d1=1;
  uint d0=PARAMS.d0, d1=PARAMS.d1, da=mdp.Pxax.d1, dy=mdp.Pyxa.d0, dx=mdp.Pxax.d0;
  FSC_lev1 fsc;
#if 0
  oneNodeOneAction(fsc.Pa0,da,d0,1.,1.,100.);
  zeroNodeStart(fsc.P0,d0);
  generalNodeTransitions(fsc.P0y0,d0,dy,.1,.1,.0);
#else  
  //initialize exactly as for go2 (with exactly same random numbers)
  FSC_lev2 fsc2;
  fsc2.P01y0.tensorResize(TUP(d0,d1,dy,d0));
  fsc2.P1y01.tensorResize(TUP(d1,dy,d0,d1));
  if(!PARAMS.adaptP0) zeroNodeStart(fsc2.P0,d0); else zeroNodeStart(fsc2.P0,d0,1.,1.);
  zeroNodeStart(fsc2.P1,d1);
  oneNodeOneAction(fsc2.Pa0,da,d0,1.,1.,100.);
  generalNode0Transition(fsc2.P01y0,d0,d1,dy,.1,.1,0.);
  generalNode1Transition(fsc2.P1y01,d1,dy,d0,.1,.1,1.);
  collapse2levelFSC(fsc,fsc2);
#endif

  // init V0x for use in EvalPolicy
  // V0x is the value function, i.e., V[q,s]
  arr V0x(d0, dx);
  V0x = 0.0;
  
  //prepare output file
  ofstream out(STRING("data/OUT1-"<<PARAMS.d0<<"-"<<PARAMS.d1<<"-"<<PARAMS.problem<<"-"<<PARAMS.Tcut<<"-"<<PARAMS.seed));
     
  //iterate EM
  double tic=MT::cpuTime();
  double Like,Val=0.;
  for(uint k=0;k<PARAMS.EMiters;k++){
    cout <<k <<' ';
    Like=EMstep_lev1_align (mdp,fsc,false,PARAMS.Tcut);
    if(PARAMS.valueCheckInterval && !((k+1)%PARAMS.valueCheckInterval)) Val=EvalPolicy(V0x, mdp, fsc);
    out <<k <<' ' <<MT::getTimer(0,tic) <<' ' <<Like <<' ' <<Val <<endl;
    // Uncomment to get AMPL-compatible output
    if(PARAMS.SetAMPLPolicy) SetAMPLPolicy(fsc.P0y0, fsc.Pa0);
  }
  cout <<"\n\ntotal time = " <<MT::getTimer(0,tic) <<endl;
}


void go_lev2(const char *problem,bool hierarchy=false){
  //load the problem
  MDP mdp;
  if(!strcmp(problem,"simpleMaze")){
    generateProblem(simpleMaze,mdp); //tinyMaze //heavenAndHell
    mdp.gamma=.9;
  }else{
    cout <<"reading POMDP file: " <<problem <<endl;
    loadPOMDP(problem, mdp);
    //mazeD0=Px.N; mazeD1=1;
    mazeD0=4; mazeD1=4;
  }
  
  //init policy
  uint d0=PARAMS.d0,d1=PARAMS.d1,da=mdp.Pxax.d1,dy=mdp.Pyxa.d0;
  FSC_lev2 fsc;
  fsc.P01y0.tensorResize(TUP(d0,d1,dy,d0));
  fsc.P1y01.tensorResize(TUP(d1,dy,d0,d1));
    //start
  
  if(!PARAMS.adaptP0) zeroNodeStart(fsc.P0,d0); else zeroNodeStart(fsc.P0,d0,1.,1.);
  zeroNodeStart(fsc.P1,d1);
  oneNodeOneAction(fsc.Pa0,da,d0,1.,1.,100.);
  generalNode0Transition(fsc.P01y0,d0,d1,dy,.1,.1,0.);
  generalNode1Transition(fsc.P1y01,d1,dy,d0,.1,.1,1.);

  /*
  arr T01y0; T01y0.tensorResize(TUP(d0,d1,dy,d0));
  arr T1y01; T1y01.tensorResize(TUP(d1,dy,d0,d1));
  if(hierarchy){
    uint exits=2;
    // notes to hierarchy:
    // - we declare `exits' nodes as exit states (another parameter...)
    // - we fix that the LAST `exits' nodes (n=d0-exits,..,d0-1) are exit states
    // - that also implies that the first node (initial node) is not an exit state!
    // - so far we only introduce structure in the n1 transitions
    makeHierarchyTemplate(exits,T01y0,T1y01,d0,d1,dy);
    fsc.P1y01 *= T1y01;
    fsc.P1y01.tensorCondNormalize(1);
  }
  */
  uint exits;
  exits=TUP(da,d1,d0/2).min();
  if(hierarchy){
    initHierarchical(exits,fsc);
  }else{
    fsc.hierarchical=false;
  }
  if(PARAMS.FixPolicy) {
    // Fix policy
    if(fsc.P01y0.d1==1 && fsc.P01y0.d0==10) fixPolicy2(fsc);
    // Fix H-policy
    else if(hierarchy && fsc.PE_01.d0==4 && fsc.PE_01.d1==4) fixPolicyH(fsc);
    else cout << "Not fixing policy\n";
  } 

  // init V0x for use in EvalPolicy
  // V0x is the value function, i.e., V[q,s]
  arr V_01_x(d0*d1, mdp.Pxax.d2);
  V_01_x = 0.0;
  FSC_lev1 fsc1;
  
  //prepare output file
  MT::String filename;
  filename <<"data/OUT";
  if(!hierarchy) filename<<"2"; else filename<<"H";
  if(!PARAMS.EscapeLocalMin) filename<<"n"; else filename<<"e";
  filename <<"-"<<PARAMS.d0<<"-"<<PARAMS.d1<<"-"<<PARAMS.problem<<"-"<<PARAMS.Tcut<<"-"<<PARAMS.seed;
  cout <<"output filename = " <<filename <<endl;
  ofstream out(filename);
  
  //iterate EM
  double tic=MT::cpuTime();
  double ticEval;
  double Like,Val=0.;
  std::list<double> LikeHistory;
  for(uint k=0;k<PARAMS.EMiters;k++){
    cout <<k <<' ';
    Like=EMstep_lev2_align_struct2(mdp,fsc,PARAMS.Tcut,false,PARAMS.maxMstep,PARAMS.adaptP0);
    ticEval=MT::cpuTime();
    if(PARAMS.valueCheckInterval && !((k+1)%PARAMS.valueCheckInterval)){
      collapse2levelFSC(fsc1,fsc);
      Val=EvalPolicy(V_01_x, mdp, fsc1);
    }

    tic+=MT::cpuTime()-ticEval;
    //if(PARAMS.SetAMPLPolicy) SetAMPLPolicy(fsc.P_01_y_01_, fsc.Pa01_);
    out <<k <<' ' <<MT::getTimer(0,tic) <<' ' <<Like <<' ' <<Val <<endl;

    if(PARAMS.EscapeLocalMin) // detect when reaching a local optima
      detectLocalOpt(Like, LikeHistory, fsc, exits, V_01_x, hierarchy);

  }
  cout <<"\n\ntotal time = " <<MT::getTimer(0,tic) <<endl;
  if(PARAMS.EscapeLocalMin) {
	if(hierarchy) {
		cout << "Final number of nodes: d0=" << fsc.P0.d0 
		                          << ", d1=" << fsc.P1.d0 << endl;
        }
  }
  if(PARAMS.OutputFSC) {
    if(fsc.P01y0.d[1] == 1) {
      cout << "Outputting FSC in Dot format\n";
      // TODO: copy fsc.P01y0 instead of actually changing it.
      fsc.P01y0.reshape(d0,dy,d0);
      OutputDot(filename+".dot",
		PARAMS.problem,
                fsc.Pa0,
                fsc.P01y0);
      fsc.P01y0.reshape(d0,d1,dy,d0);
    }
    else if(hierarchy)    
      OutputDotH(filename+".dot",PARAMS.problem,
                 fsc.Pa0, fsc.Pe0, fsc.PE_01, fsc.P_E_0y0, fsc.PE_1y1, fsc.P_E_11);
  }

  //while(1){  EMstep_lev2_align(mdp,fsc,true,T); }
}

    
int main(int argn,char** argv){
 
  MT::init(argn,argv);
  MT::getParameter(PARAMS.seed,"seed",(uint)0);
  MT::getParameter(PARAMS.d0,"d0");
  MT::getParameter(PARAMS.d1,"d1",(uint)0);
  MT::getParameter(PARAMS.valueCheckInterval,"valueCheckInterval",(uint)100);
  MT::getParameter(PARAMS.problem,"problem");
  MT::getParameter(PARAMS.Tcut,"Tcut",(uint)100);
  MT::getParameter(PARAMS.EMiters,"EMiters",(uint)200);
  MT::getParameter(PARAMS.OutputFSC,"OutputFSC",(uint)0);
  MT::getParameter(PARAMS.FixPolicy,"FixPolicy",(uint)0);
  MT::getParameter(PARAMS.EscapeLocalMin,"EscapeLocalMin",(uint)0);  
  MT::getParameter(PARAMS.SetAMPLPolicy,"SetAMPLPolicy",(uint)0);
  MT::getParameter(PARAMS.maxMstep,"maxMstep",true);
  MT::getParameter(PARAMS.adaptP0,"adaptP0",false);
  rnd.seed(PARAMS.seed);

  cout <<std::setprecision(5);
  
  int mode=0;
  if(argn>1) mode=atoi(argv[1]);
  switch(mode){
  case 1: convertProblem(PARAMS.problem); break;
  case 2: scanArrFile(PARAMS.problem); break;
  case 3: go_lev1(PARAMS.problem);     break;
  case 4: go_lev2(PARAMS.problem);     break;
  case 5: go_lev2(PARAMS.problem,true);break;
  default: NIY;
  }
  
  return 0;
}
