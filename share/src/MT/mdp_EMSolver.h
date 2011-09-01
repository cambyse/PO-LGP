#ifndef MT_solver_h
#define MT_solver_h

#include "mdp.h"

namespace mdp {

enum FscType { FscPlain, FscHierarchical, FscReactive };

struct EMSolver {
  //----- public options of the solver
  //general
  MT::String problemFile;    //!< file name of the problem
  MT::String fscFile;        //!< file name for the controller
  MT::String outputPrefix;   //!< path-prefix for the outputfile
  uint seed;                 //!< random seed
  uint EMiterations;         //!< #iterations when loop() is called
  uint evaluationCheckHorizon;  //!< use a (flat) policy evaluation in each iteration to check the reward
  //controller options
  FscType fscType;           //!< fsc type [[todo make an enum]]
  uintA levels;              //!< array with #nodes in each level
  //Mstep options
  MstepType  mstepType;      //!< type of Mstep
  double mstepRate;          //!< Mstep ``convergence rate''
  double mstepNoise;         //!< Mstep noise level
  //Estep options
  bool estepStructured;      //!< use structured inference instead of collapsing everything to a single variable
  bool estepIncremental;     //!< reuse the alpha and beta from the previous iteration (-> can use much smaller horizon)
  uint estepHorizon;         //!< number of time slices propagated during the E-step
  //obsolete (still here for testing)
  bool obsolete_forceLevel1;
  
  //----- internal variables
  MT::String outfilename;
  ofstream outfile;
  arr alpha, beta;
  double tic;
  MDP_structured mdps;
  FSC_structured fsc;
  uint k;
  
  //----- methods
  void getParameters();
  void reportParameters(std::ostream& os);
  
  void clear(){ alpha.clear(); beta.clear(); clearMDP(mdps); clearFSC(fsc); }
  void initProblem();
  void initFsc();
  void resetTimer();
  void step();
  void loop();
  void gnuplot(bool byTime);
  
  void obsolete_loop_lev12(); //!< old routines for 1 or 2 levels (very useful for testing!)
  
};

}//end of namespace mdp

#ifdef  MT_IMPLEMENTATION
#include "mdp_EMSolver.cpp"
#endif

#endif
