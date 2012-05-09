#include "mdp_EMSolver.h"

void mdp::EMSolver::getParameters(){
  MT::getParameter(problemFile, "problemFile", MT::String("paint.95.POMDP.arr"));
  MT::getParameter(fscFile, "fscFile", MT::String("z.fsc"));
  MT::getParameter(outputPrefix, "outputPrefix", MT::String("data/test"));
  MT::getParameter(seed, "seed", (uint)0);
  MT::getParameter(EMiterations, "EMiterations", (uint)200);
  MT::getParameter(evaluationCheckHorizon, "evaluationCheckHorizon", (uint)0);
  
  MT::getParameter((int&)fscType, "fscType", 0);
  MT::getParameter(levels, "levels", TUP(10));
  
  MT::getParameter((int&)mstepType, "mstepType", 0);
  MT::getParameter(mstepRate, "mstepRate", .3);
  MT::getParameter(mstepNoise, "mstepNoise", 1e-5);
  
  MT::getParameter(estepHorizon, "estepHorizon", (uint)200);
  MT::getParameter(estepStructured, "estepStructured", true);
  MT::getParameter(estepIncremental, "estepIncremental", true);
  
  MT::getParameter(obsolete_forceLevel1, "forceLevel1", false);;
}

void mdp::EMSolver::reportParameters(std::ostream& os){
  cout <<"\n *** EMSolver parameter setting"
       <<"\ngeneral:"
       <<"\n  problemFile  = " <<problemFile
       <<"\n  fscFile      = " <<fscFile
       <<"\n  outputPrefix = " <<outputPrefix
       <<"\n  seed         = " <<seed
       <<"\n  EMiterations = " <<EMiterations
       <<"\n  evaluationCheckHorizon = " <<evaluationCheckHorizon
       <<"\ncontroller parameters:"
       <<"\n  fscType = " <<fscType
       <<"\n  levels  = " <<levels
       <<"\nM-step parameters:"
       <<"\n  mstepType  = " <<mstepType
       <<"\n  mstepRate  = " <<mstepRate
       <<"\n  mstepNoise = " <<mstepNoise
       <<"\nE-step parameters:"
       <<"\n  estepIncremental = " <<estepIncremental
       <<"\n  estepStructured  = " <<estepStructured
       <<"\n  estepHorizon     = " <<estepHorizon
       <<"\nobseolete:"
       <<"\n  forceLevel1 = " <<obsolete_forceLevel1
       <<endl;
}

void mdp::EMSolver::initProblem(){
  outfilename.clear() <<outputPrefix;
  //MT::IOraw=true;
  //levels.write(outfilename, "-");
  //MT::IOraw=false;
  outfilename <<"." <<seed;
  cout <<"output filename = " <<outfilename <<endl;
  outfile.close();
  MT::open(outfile, outfilename);
  
  readMDP(mdps, problemFile);
}

void mdp::EMSolver::initFsc(){
  rnd.seed(seed);
  switch(fscType){
    case FscPlain:
      //standardInitFsc_structured_levels(fsc, mdp, levels);
      standardInitFsc_structured_levels(fsc, mdps, levels);
      break;
    case FscHierarchical:
      //standardInitFsc_structured_hierarchical(fsc, mdp, levels);
      NIY;
      break;
    case FscReactive:
      //standardInitFsc_structured_reactive(fsc, mdp, levels);
      NIY;
      break;
  }
  //ofstream z(outfilename+".init-fsc");
  //writeFSC_fg(fsc, z, false);
  alpha.clear();
  beta.clear();
  outfile <<endl;
  k=0;
}

void mdp::EMSolver::resetTimer(){
  tic=MT::cpuTime();
}

void mdp::EMSolver::step(){
  cout <<k <<' ';
  cout <<std::setprecision(8);
  double R;
  R=pomdpEM_structured(mdps, fsc,
                       estepHorizon, estepStructured, alpha.N>0 && estepIncremental,
                       mstepType, mstepRate, mstepNoise,
                       false, &alpha, &beta, &cout);
  outfile <<k <<' ' <<MT::timerRead(false, tic) <<' ' <<R <<endl;
  if(false && evaluationCheckHorizon){ //NIY...
    uint horizon=evaluationCheckHorizon;
    if(horizon==1) horizon=2*estepHorizon;
    FSC_lev1 fsc1;
    MDP mdp;
    collapseFSC(fsc1, fsc);
    collapseToFlat(mdp, mdps);
    cout <<"evaluation = " <<evaluateFsc1(fsc1, mdp, horizon) <<endl;
  }
  k++;
}

void mdp::EMSolver::loop(){
  tic=MT::cpuTime();
  for(; k<EMiterations;) step();
  //cout <<"final R = " <<R <<"\ntotal time = " <<MT::timerRead(false, tic) <<endl;
  cout <<"total time = " <<MT::timerRead(false, tic) <<endl;
}

void mdp::EMSolver::gnuplot(bool byTime){
  MT::String cmd;
  if(byTime) cmd <<"plot '" <<outfilename <<"' us 2:3";
  else       cmd <<"plot '" <<outfilename <<"' us 1:3";
  cout <<"gnuplot command: " <<cmd <<endl;
  ::gnuplot(cmd.p);
}

void mdp::EMSolver::obsolete_loop_lev12(){
  MDP mdp;
  mdp::collapseToFlat(mdp, mdps);
  
  //init policy
  FSC_lev1 fsc1;
  FSC_lev2 fsc2;
  rnd.seed(seed);
  CHECK(levels.N==1 || levels.N==2, "old solver can only handle 1 or 2 levels!");
  if(levels.N==1){
    standardInitFsc1(fsc1, mdp, levels(0));
    writeFSC_lev1(STRING(outfilename <<".init-fsc1"), fsc1, false);
  }
  if(levels.N==2){
    standardInitFsc2(fsc2, mdp, levels(0), levels(1), fscType==FscHierarchical);
    writeFSC_lev2(STRING(outfilename <<".init-fsc2"), fsc2, false);
  }
  
  //iterate EM
  double R=0.;
  tic=MT::cpuTime();
  if(levels.N==1 || obsolete_forceLevel1){ //use specialized flat controller optimization
    if(levels.N==2) collapse2levelFSC(fsc1, fsc2);
    for(uint k=0; k<EMiterations; k++){
      if(evaluationCheckHorizon){
        uint horizon=evaluationCheckHorizon;
        if(horizon==1) horizon=2*estepHorizon;
        cout <<"evaluation = " <<evaluateFsc1(fsc1, mdp, horizon) <<endl;
      }
      cout <<k <<' ';
      //cout <<std::setprecision(8);
      R=pomdpEM_lev1(mdp, fsc1, estepHorizon, estepStructured, alpha.N>0 && estepIncremental,
                     (MstepType)mstepType, mstepRate, mstepNoise,
                     false, &alpha, &beta, &cout);
      outfile <<k <<' ' <<MT::timerRead(false, tic) <<' ' <<R <<endl;
    }
  }else{
    for(uint k=0; k<EMiterations; k++){
      if(evaluationCheckHorizon){
        uint horizon=evaluationCheckHorizon;
        if(horizon==1) horizon=2*estepHorizon;
        collapse2levelFSC(fsc1, fsc2);
        cout <<"evaluation = " <<evaluateFsc1(fsc1, mdp, horizon) <<endl;
      }
      cout <<k <<' ';
      R=pomdpEM_lev2(mdp, fsc2, estepHorizon, estepStructured, (MstepType)mstepType, false, &cout);
      outfile <<k <<' ' <<MT::timerRead(false, tic) <<' ' <<R <<endl;
    }
  }
  cout <<"final R = " <<R <<"\ntotal time = " <<MT::timerRead(false, tic) <<endl;
}
