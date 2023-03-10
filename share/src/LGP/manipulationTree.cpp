/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "manipulationTree.h"
#include <MCTS/solver_PlainMC.h>
#include <KOMO/komo.h>
#include <Kin/switch.h>

#define DEBUG(x) //x
#define DEL_INFEASIBLE(x) //x

uint COUNT_kin=0;
uint COUNT_evals=0;
uintA COUNT_opt=consts<uint>(0, 4);

ManipulationTree_Node::ManipulationTree_Node(mlr::KinematicWorld& kin, FOL_World& _fol, uint levels)
    : parent(NULL), step(0),
    fol(_fol), folState(NULL), folDecision(NULL), folAddToState(NULL),
    startKinematics(kin), effKinematics(),
    L(levels), bound(0.),
    rootMC(NULL), mcStats(NULL), mcCount(0), mcCost(0.) {
  //this is the root node!
  fol.reset_state();
  folState = fol.createStateCopy();
  cost = zeros(L);
  constraints = zeros(L);
  count = consts<uint>(0, L);
  count(l_symbolic) = 1;
  feasible = consts<byte>(true, L);
  komoProblem = consts<KOMO*>(NULL, L);
  opt.resize(L);
  bound=0.;
}

ManipulationTree_Node::ManipulationTree_Node(ManipulationTree_Node* parent, MCTS_Environment::Handle& a)
  : parent(parent), fol(parent->fol),
     folState(NULL), folDecision(NULL), folAddToState(NULL),
    startKinematics(parent->startKinematics), effKinematics(),
    L(parent->L), bound(0.),
    rootMC(NULL), mcStats(NULL), mcCount(0), mcCost(0.) {
  step=parent->step+1;
  parent->children.append(this);
  fol.setState(parent->folState, parent->step);
  CHECK(a,"giving a 'NULL' shared pointer??");
  ret = fol.transition(a);
  time = parent->time + ret.duration;
  isTerminal = fol.successEnd;
  if(fol.deadEnd) isInfeasible=true;
  folState = fol.createStateCopy();
  folAddToState = NULL; //fresh creation -> notion to add
  folDecision = folState->getNode("decision");
  decision = a;
  rootMC = parent->rootMC;
  cost = zeros(L);
  count = consts<uint>(0, L);
  count(l_symbolic) = 1;
  feasible = consts<byte>(true, L);
  cost(l_symbolic) = parent->cost(l_symbolic) - 0.1*ret.reward; //cost-so-far
  constraints = zeros(L);
  komoProblem = consts<KOMO*>(NULL, L);
  opt.resize(L);
  bound = parent->bound - 0.1*ret.reward;
//  h(l_symbolic) = 0.; //heuristic
}

ManipulationTree_Node::~ManipulationTree_Node(){
    for(ManipulationTree_Node *ch:children) delete ch;
    for(KOMO* k:komoProblem) if(k) delete k;
}

void ManipulationTree_Node::expand(int verbose){
    if(isExpanded) return; //{ LOG(-1) <<"MNode '" <<*this <<"' is already expanded"; return; }
  CHECK(!children.N,"");
  if(isTerminal) return;
  fol.setState(folState, step);
  int tmp=fol.verbose;
  fol.verbose=verbose;
  auto actions = fol.get_actions();
  fol.verbose=tmp;
  for(FOL_World::Handle& a:actions){
//    cout <<"  EXPAND DECISION: " <<*a <<endl;
    new ManipulationTree_Node(this, a);
  }
  if(!children.N) isTerminal=true;
  isExpanded=true;
}

arr ManipulationTree_Node::generateRootMCRollouts(uint num, int stepAbort, const mlr::Array<MCTS_Environment::Handle>& prefixDecisions){
  CHECK(!parent, "generating rollouts needs to be done by the root only");

  fol.reset_state();
//  cout <<"********\n *** MC from STATE=" <<*fol.state->isNodeOfGraph <<endl;
  if(!rootMC){
    rootMC = new PlainMC(fol);
    rootMC->verbose = 0;
  }

  arr R;

  for(uint k=0;k<num;k++){
    rootMC->initRollout(prefixDecisions);
    fol.setState(folState);
    double r = rootMC->finishRollout(stepAbort);
    R.append( r );
  }

  return R;
}

void ManipulationTree_Node::addMCRollouts(uint num, int stepAbort){
  //-- collect decision path
  ManipulationTree_NodeL treepath = getTreePath();
  mlr::Array<MCTS_Environment::Handle> prefixDecisions(treepath.N-1);
  for(uint i=1;i<treepath.N;i++)
    prefixDecisions(i-1) = treepath(i)->decision;

//  cout <<"DECISION PATH = "; listWrite(prefixDecisions); cout <<endl;

#if 0
  arr R = generateRootMCRollouts(num, stepAbort, prefixDecisions);
#else
  arr R;
  for(uint k=0;k<num;k++){
    rootMC->initRollout(prefixDecisions);
    fol.setState(folState);
    double r = rootMC->finishRollout(stepAbort);
    R.append( r );
  }
#endif

  for(ManipulationTree_Node* n:treepath){
    if(!n->mcStats) n->mcStats = new MCStatistics;
    for(auto& r:R){
      n->mcStats->add(r);
      n->mcCost = - n->mcStats->X.first();
    }
  }

  mcCount += num;
//  mcStats->report();
//  auto a = rootMC->getBestAction();
  //  cout <<"******** BEST ACTION " <<*a <<endl;
}

void ManipulationTree_Node::optLevel(uint level, bool collisions){
  komoProblem(level) = new KOMO();
  KOMO& komo(*komoProblem(level));

  //cout <<"########## OPTIM lev " <<level <<endl;

  //-- prepare the komo problem
  switch(level){
  case 1:{
      //pose: propagate eff kinematics
      if(!parent) effKinematics = startKinematics;
      else{
        if(!parent->effKinematics.q.N){
            LOG(-1) <<"I can't compute a pose when no pose was comp. for parent (I need the effKin)";
            return;
        }
        effKinematics = parent->effKinematics;
      }

      komo.setModel(effKinematics, false);
      komo.setTiming(1., 2, 5., 1);

      komo.setHoming(-1., -1., 1e-2);
      komo.setSquaredQVelocities(.5); //IMPORTANT: do not penalize transitions of from prefix to x_{0} -> x_{0} is 'loose'
      //komo.setFixEffectiveJoints(-1., -1., 1e2);
      komo.setFixSwitchedObjects(-1., -1., 1e2);
      komo.setSquaredQuaternionNorms();

      komo.setAbstractTask(0., *folState);
  } break;
  case 2:{
      komo.setModel(startKinematics, false);
      komo.setTiming(time, 2, 5., 1);

      komo.setHoming(-1., -1., 1e-2);
      komo.setSquaredQVelocities();
      komo.setFixEffectiveJoints(-1., -1., 1e2);
      komo.setFixSwitchedObjects(-1., -1., 1e2);
      komo.setSquaredQuaternionNorms();

      for(ManipulationTree_Node *node:getTreePath()){
        komo.setAbstractTask((node->parent?node->parent->time:0.), *node->folState);
      }
  } break;
  case 3:{
      komo.setModel(startKinematics, collisions);
      uint stepsPerPhase = mlr::getParameter<uint>("LGP/stepsPerPhase", 10);
      komo.setTiming(time+.5, stepsPerPhase, 5., 2);

      komo.setHoming(-1., -1., 1e-2);
      komo.setSquaredQAccelerations();
      komo.setFixEffectiveJoints(-1., -1., 1e2);
      komo.setFixSwitchedObjects(-1., -1., 1e2);
      komo.setSquaredQuaternionNorms();
      if(collisions) komo.setCollisions(false);

      for(ManipulationTree_Node *node:getTreePath()){
        komo.setAbstractTask((node->parent?node->parent->time:0.), *node->folState);
      }
  } break;
  }

  //-- optimize
  DEBUG( FILE("z.fol") <<fol; )
  DEBUG( komo.getReport(false, 1, FILE("z.problem")); )
  komo.reset();
  try{
//      komo.verbose=3;
    komo.run();
  } catch(const char* msg){
    cout <<"KOMO FAILED: " <<msg <<endl;
  }
  COUNT_evals += komo.opt->newton.evals;
  COUNT_kin += mlr::KinematicWorld::setJointStateCount;
  COUNT_opt(level)++;
  count(level)++;

  DEBUG( komo.getReport(false, 1, FILE("z.problem")); );
//  komo.checkGradients();

  Graph result = komo.getReport((komo.verbose>0 && level==3));
  DEBUG( FILE("z.problem.cost") <<result; )
  double cost_here = result.get<double>({"total","sqrCosts"});
  double constraints_here = result.get<double>({"total","constraints"});
  bool feas = (constraints_here<.5);

  //-- post process komo problem for level==1
  if(level==1){
      cost_here -= 0.1*ret.reward; //account for the symbolic costs
      if(parent) cost_here += parent->cost(level); //this is sequentially additive cost

      effKinematics = *komo.configurations.last();

      for(mlr::KinematicSwitch *sw: komo.switches){
    //    CHECK_EQ(sw->timeOfApplication, 1, "need to do this before the optimization..");
        if(sw->timeOfApplication>=2) sw->apply(effKinematics);
      }

      effKinematics.reset_q();
      effKinematics.calc_q();
      DEBUG( effKinematics.checkConsistency(); )
  }else{
      cost_here += cost(l_symbolic); //account for the symbolic costs
  }

  //-- read out and update bound
  //update the bound
  if(feas){
    if(count(level)==1/*&& count({2,-1})==0 (also no higher levels)*/ || cost_here<bound) bound=cost_here;
  }

  if(count(level)==1 || cost_here<cost(level)){
    cost(level) = cost_here;
    constraints(level) = constraints_here;
    feasible(level) = feas;
    opt(level) = komo.x;
  }

  if(!feasible(level))
    labelInfeasible();

#if 0 //DEBUG
  for(uint i=0;i<komo.configurations.N;i++){
      FILE(STRING("z.config."<<i)) <<*komo.configurations(i);
  }
#endif
}

//void ManipulationTree_Node::createEffKinematics(){
//  KOMO komo;

//  CHECK(!effKinematics.q.N, "has been created before");

//  if(!parent) effKinematics = startKinematics;
//  else{
//      if(!parent->effKinematics.q.N){
//          LOG(-1) <<"I can't compute a pose when no pose was comp. for parent (I need the effKin)";
//          return;
//      }
//      effKinematics = parent->effKinematics;
//  }

//  komo.setModel(effKinematics);
//  komo.setAbstractTask(0., *folState);


//  effKinematics = *komo.configurations.last();

//  for(uint t=0;t<komo.T;t++){
//      for(mlr::KinematicSwitch *sw: komo.switches){
//          if(sw->timeOfApplication==t) sw->apply(effKinematics);
//      }
//  }
//  effKinematics.topSort();
//  DEBUG( effKinematics.checkConsistency(); )
//          effKinematics.getJointState();
//}

#ifdef OLD
void ManipulationTree_Node::solvePoseProblem(){
  uint level=1;

  //reset the effective kinematics:
  if(parent && !parent->effKinematics.q.N){
    MLR_MSG("parent needs to have computed the pose first!");
    return;
  }
  if(!parent) effKinematics = startKinematics;
  else effKinematics = parent->effKinematics;

  //-- collect 'path nodes'
  ManipulationTree_NodeL treepath = getTreePath();

  poseProblem = new KOMO();
  KOMO& komo(*poseProblem);
  komo.setModel(effKinematics);
  komo.setTiming(1., 2, 5., 1, false);

  komo.setHoming(-1., -1., 1e-1); //gradient bug??
  komo.setSquaredQVelocities();
//  komo.setFixEffectiveJoints(-1., -1., 1e3);
  komo.setFixSwitchedObjects(-1., -1., 1e3);

  komo.setAbstractTask(0., *folState);
//  for(mlr::KinematicSwitch *sw: poseProblem->switches){
//    sw->timeOfApplication=2;
//  }

  DEBUG( FILE("z.fol") <<fol; )
  DEBUG( komo.getReport(false, 1, FILE("z.problem")); )
  komo.reset();
  try{
    komo.run();
  } catch(const char* msg){
    cout <<"KOMO FAILED: " <<msg <<endl;
  }
  COUNT_evals += komo.opt->newton.evals;
  COUNT_kin += mlr::KinematicWorld::setJointStateCount;
  COUNT_opt(level)++;
  count(level)++;

  DEBUG( komo.getReport(false, 1, FILE("z.problem")); )

  Graph result = komo.getReport();
  DEBUG( FILE("z.problem.cost") <<result; )
  double cost_here = result.get<double>({"total","sqrCosts"});
  double constraints_here = result.get<double>({"total","constraints"});
  bool feas = (constraints_here<.5);

  cost_here -= 0.1*ret.reward; //account for the symbolic costs
  if(parent) cost_here += parent->cost(level); //this is sequentially additive cost

  //update the bound
  if(feas){
    if(count(level)==1/*&& count({2,-1})==0 (also no higher levels)*/ || cost_here<bound) bound=cost_here;
  }

  if(count(level)==1 || cost_here<cost(level)){
    cost(level) = cost_here;
    constraints(level) = constraints_here;
    feasible(level) = feas;
    pose = komo.x;
  }

  if(!feasible(level))
    labelInfeasible();

  effKinematics = *komo.configurations.last();

  for(mlr::KinematicSwitch *sw: komo.switches){
//    CHECK_EQ(sw->timeOfApplication, 1, "need to do this before the optimization..");
    if(sw->timeOfApplication>=2) sw->apply(effKinematics);
  }
  effKinematics.topSort();
  DEBUG( effKinematics.checkConsistency(); )
  effKinematics.getJointState();
}

void ManipulationTree_Node::solveSeqProblem(int verbose){
  uint level=2;

  if(!step){ feasible(level)=true; return; } //there is no sequence to compute

  //-- collect 'path nodes'
  ManipulationTree_NodeL treepath = getTreePath();

  seqProblem = new KOMO();
  KOMO& komo(*seqProblem);
  komo.setModel(startKinematics);
  komo.setTiming(time, 2, 5., 1, false);

  komo.setHoming(-1., -1., 1e-1); //gradient bug??
  komo.setSquaredQVelocities();
  komo.setFixEffectiveJoints(-1., -1., 1e3);
  komo.setFixSwitchedObjects(-1., -1., 1e3);

  for(ManipulationTree_Node *node:treepath){
    komo.setAbstractTask((node->parent?node->parent->time:0.), *node->folState);
  }

  DEBUG( FILE("z.fol") <<fol; )
  DEBUG( komo.getReport(false, 1, FILE("z.problem")); )
  komo.reset();
  try{
    komo.run();
  } catch(const char* msg){
    cout <<"KOMO FAILED: " <<msg <<endl;
  }
  COUNT_evals += komo.opt->newton.evals;
  COUNT_kin += mlr::KinematicWorld::setJointStateCount;
  COUNT_opt(level)++;
  count(level)++;

  DEBUG( komo.getReport(false, 1, FILE("z.problem")); )
//  komo.checkGradients();

  Graph result = komo.getReport();
  DEBUG( FILE("z.problem.cost") <<result; )
  double cost_here = result.get<double>({"total","sqrCosts"});
  double constraints_here = result.get<double>({"total","constraints"});
  bool feas = (constraints_here<.5);

  cost_here += cost(l_symbolic); //account for the symbolic costs

  //update the bound
  if(feas){
    if(!count(level)/*actually !count({1,-1}) (also no higher levels)*/ || cost_here<bound) bound=cost_here;
  }

  if(!seq.N || cost_here<cost(level)){
    cost(level) = cost_here;
    constraints(level) = constraints_here;
    feasible(level) = (constraints_here<.5);
    seq = komo.x;
  }

  if(!feasible(level))
    labelInfeasible();
}

void ManipulationTree_Node::solvePathProblem(uint microSteps, int verbose){
  uint level=3;

  if(!step){ feasible(level)=true; return; } //there is no path to compute

  //-- collect 'path nodes'
  ManipulationTree_NodeL treepath = getTreePath();

  pathProblem = new KOMO();
  KOMO& komo(*pathProblem);
  komo.setModel(startKinematics);
  komo.setTiming(time, microSteps, 5., 2, false);

  komo.setHoming(-1., -1., 1e-2); //gradient bug??
  komo.setSquaredQAccelerations();
  komo.setFixEffectiveJoints(-1., -1., 1e3);
  komo.setFixSwitchedObjects(-1., -1., 1e3);

  for(ManipulationTree_Node *node:treepath){
    komo.setAbstractTask((node->parent?node->parent->time:0.), *node->folState);
  }

  DEBUG( FILE("z.fol") <<fol; )
  DEBUG( komo.getReport(false, 1, FILE("z.problem")); )
  komo.reset();
  try{
    komo.run();
  } catch(const char* msg){
    cout <<"KOMO FAILED: " <<msg <<endl;
  }
  COUNT_evals += komo.opt->newton.evals;
  COUNT_kin += mlr::KinematicWorld::setJointStateCount;
  COUNT_opt(level)++;
  count(level)++;

  DEBUG( komo.getReport(false, 1, FILE("z.problem")); )
//  komo.checkGradients();

  Graph result = komo.getReport();
  DEBUG( FILE("z.problem.cost") <<result; )
  double cost_here = result.get<double>({"total","sqrCosts"});
  double constraints_here = result.get<double>({"total","constraints"});
  bool feas = (constraints_here<.5);

  cost_here += cost(l_symbolic); //account for the symbolic costs

  //update the bound
  if(feas){
    if(!count(level)/*actually !count({1,-1}) (also no higher levels)*/ || cost_here<bound) bound=cost_here;
  }

  if(!path.N || cost_here<cost(level)){
    cost(level) = cost_here;
    constraints(level) = constraints_here;
    feasible(level) = (constraints_here<.5);
    path = komo.x;
  }

  if(!feasible(level))
    labelInfeasible();
}
#endif

void ManipulationTree_Node::setInfeasible(){
  isInfeasible = true;
  for(ManipulationTree_Node *n:children) n->setInfeasible();
}

void ManipulationTree_Node::labelInfeasible(){
  setInfeasible();

  //-- remove children
//  ManipulationTree_NodeL tree;
//  getAllChildren(tree);
//  for(ManipulationTree_Node *n:tree) if(n!=this) delete n; //TODO: memory leak!
  DEL_INFEASIBLE( children.clear(); )

  //-- create a literal that is equal to the decision literal (tuple) plus an 'INFEASIBLE' prepended
  NodeL symbols = folDecision->parents;
  symbols.prepend( fol.KB.getNode({"INFEASIBLE"}));
//  cout <<"\n *** LABELLING INFEASIBLE: "; listWrite(symbols); cout <<endl;

  //-- find the right parent-of-generalization
  ManipulationTree_Node* branchNode = this;
  while(branchNode->parent){
    bool stop=false;
    for(Node *fact:branchNode->folState->list()){
      if(fact->keys.N && fact->keys.last()=="block"){
        if(tuplesAreEqual(fact->parents, symbols)){
          CHECK(fact->isOfType<bool>() && fact->keys.first()=="block", "");
          stop=true;
          break;
        }
      }
    }
    if(stop) break;
    branchNode = branchNode->parent;
  }

  //add the infeasible-literal as an 'ADD' command to the branch node
  if(!branchNode->folAddToState){
    branchNode->folAddToState = &fol.KB.newSubgraph({"ADD"}, {branchNode->folState->isNodeOfGraph})->value;
  }
  branchNode->folAddToState->newNode<bool>({}, symbols, true);

//  ManipulationTree_Node *root=getRoot();
  branchNode->recomputeAllFolStates();
//  node->recomputeAllMCStats(false);
  //TODO: resort all queues
}

ManipulationTree_NodeL ManipulationTree_Node::getTreePath() const{
  ManipulationTree_NodeL path;
  ManipulationTree_Node *node=(ManipulationTree_Node*)this;
  for(;node;){
    path.prepend(node);
    node = node->parent;
  }
  return path;
}

ManipulationTree_Node* ManipulationTree_Node::getRoot(){
  ManipulationTree_Node* n=this;
  while(n->parent) n=n->parent;
  return n;
}

ManipulationTree_Node *ManipulationTree_Node::getChildByAction(Node *folDecision){
  for(ManipulationTree_Node *ch:children){
    if(tuplesAreEqual(ch->folDecision->parents, folDecision->parents)) return ch;
  }
  LOG(-1) <<"a child with action '" <<*folDecision <<"' does not exist";
  return NULL;
}

void ManipulationTree_Node::getAll(ManipulationTree_NodeL& L){
  L.append(this);
  for(ManipulationTree_Node *ch:children) ch->getAll(L);
}

ManipulationTree_Node *ManipulationTree_Node::treePolicy_random(){
  if(isInfeasible) return NULL;
  if(isTerminal) return NULL;
  if(children.N) return children.rndElem()->treePolicy_random();
  return this;
}


bool ManipulationTree_Node::recomputeAllFolStates(){
  if(!parent){ //this is root
    folState->copy(*fol.start_state);
    if(folAddToState) applyEffectLiterals(*folState, *folAddToState, {}, NULL);
  }else{
    fol.setState(parent->folState, parent->step);
    if(fol.is_feasible_action(decision)){
      ret = fol.transition(decision);
      time = parent->time + ret.duration;
      isTerminal = fol.successEnd;
      if(fol.deadEnd){
        if(!feasible(l_seq) && !feasible(l_path)) //seq or path have already proven it feasible! Despite the logic...
          isInfeasible=true;
        return false;
      }
      folState->copy(*fol.state);
      if(folAddToState) applyEffectLiterals(*folState, *folAddToState, {}, NULL);
      folDecision = folState->getNode("decision");
    }else{
      if(!feasible(l_seq) && !feasible(l_path)) //seq or path have already proven it feasible! Despite the logic...
        isInfeasible=true;
      return false;
    }
  }
  if(children.N){
    for(uint i=children.N-1;;){
      DEL_INFEASIBLE( bool feasible = children(i)->recomputeAllFolStates(); )
      DEL_INFEASIBLE( if(!feasible) children.remove(i); )
      if(!i || !children.N) break;
      i--;
    }
  }
  DEBUG( if(!parent) FILE("z.fol") <<fol; )
  return true;
}

void ManipulationTree_Node::recomputeAllMCStats(bool excludeLeafs){
  if(!mcStats) return;
  if(!isTerminal){
    if(children.N || !excludeLeafs || isInfeasible)
      mcStats->clear();
  }
  for(ManipulationTree_Node* ch:children){
    ch->recomputeAllMCStats(excludeLeafs);
    for(double x:ch->mcStats->X) mcStats->add(x);
  }
  if(mcStats->n)
    mcCost = - mcStats->X.first();
  else
    mcCost = 100.;
}

void ManipulationTree_Node::checkConsistency(){
  //-- check that the state->parent points to the parent's state
  if(parent){
    CHECK_EQ(parent->folState->isNodeOfGraph, folState->isNodeOfGraph->parents.scalar(), "");
    CHECK_EQ(&folDecision->container, folState, "");
  }

  //-- check that each child exactly matches a decision, in same order
  if(children.N){
    fol.setState(folState, step);
    auto actions = fol.get_actions();
    CHECK_EQ(children.N, actions.size(), "");
    uint i=0;
    for(FOL_World::Handle& a:actions){
//      cout <<"  DECISION: " <<*a <<endl;
      FOL_World::Handle& b = children(i)->decision;
      CHECK_EQ(*a, *b, "children do not match decisions");
      i++;
    }
  }

  for(auto* ch:children) ch->checkConsistency();
}

void ManipulationTree_Node::write(ostream& os, bool recursive, bool path) const{
  os <<"------- NODE -------\ns=" <<step <<" t=" <<time;
  if(decision) os <<" a=" <<*decision <<endl;
  else os <<" a=<ROOT>"<<endl;

  os <<"\t state= " <<*folState->isNodeOfGraph <<endl;
  if(path){
      os <<"\t decision path:";
      ManipulationTree_NodeL _path = getTreePath();
      for(ManipulationTree_Node *nn: _path)
          if(nn->decision) os <<*nn->decision <<' '; else os <<" <ROOT> ";
      os <<endl;
  }
  os <<"\t depth=" <<step <<endl;
  os <<"\t poseCost=" <<cost(l_pose) <<endl;
  os <<"\t seqCost=" <<cost(l_seq) <<endl;
  os <<"\t pathCost=" <<cost(l_path) <<endl;
  if(recursive) for(ManipulationTree_Node *n:children) n->write(os);
}

void ManipulationTree_Node::getGraph(Graph& G, Node* n) {
  if(!n){
    n = G.newNode<bool>({"a:<ROOT>"}, NodeL(), true);
  }else{
    n = G.newNode<bool>({STRING("a:"<<*decision)}, {n}, true);
  }
  n->keys.append(STRING("s:" <<step <<" t:" <<time <<" bound:" <<bound <<" feas:" <<!isInfeasible <<" term:" <<isTerminal <<' ' <<folState->isNodeOfGraph->keys.scalar()));
  for(uint l=0;l<L;l++)
      n->keys.append(STRING("L" <<l <<" #:" <<count(l) <<" c:" <<cost(l) <<"|" <<constraints(l) <<" f:" <<feasible(l) <<" terminal:" <<isTerminal));

  if(mcStats && mcStats->n) n->keys.append(STRING("MC best:" <<mcStats->X.first() <<" n:" <<mcStats->n));
//  n->keys.append(STRING("sym  g:" <<cost(l_symbolic) <<" h:" <<h(l_symbolic) <<" f:" <<f() <<" terminal:" <<isTerminal));
  n->keys.append(STRING("MC   #" <<mcCount <<" f:" <<mcCost));
  if(folAddToState) n->keys.append(STRING("symAdd:" <<*folAddToState));
  if(note.N) n->keys.append(note);

  G.getRenderingInfo(n).dotstyle="shape=box";
  if(isInfeasible){
    if(isTerminal)  G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=violet";
    else G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=red";
  }else if(isTerminal){
    if(count(l_seq) || count(l_path)) G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=cyan";
    else G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=blue";
  }else{
    if(sum(count) - count(l_symbolic)>0) G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=green";
  }
//  if(inFringe1) G.getRenderingInfo(n).dotstyle <<" color=green";
//  if(inFringe1) G.getRenderingInfo(n).dotstyle <<" peripheries=2";
//  if(inFringe2) G.getRenderingInfo(n).dotstyle <<" peripheries=3";

//  n->keys.append(STRING("reward:" <<effPoseReward));
  for(ManipulationTree_Node *ch:children) ch->getGraph(G, n);
}

RUN_ON_INIT_BEGIN(manipulationTree)
ManipulationTree_NodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
