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


#include "action_node.h"
#include <MCTS/solver_PlainMC.h>

#define DEBUG(x) //x
#define DEL_INFEASIBLE(x) x

uint COUNT_kin=0;
uint COUNT_evals=0;
uint COUNT_poseOpt=0;
uint COUNT_seqOpt=0;
uint COUNT_pathOpt=0;

//==============KOMOFactory==============================================

void KOMOFactory::registerTask( const mlr::String & type, const SymbolGrounder & grounder )
{
  tasks_[ type ] = grounder;
}

std::shared_ptr< ExtensibleKOMO > KOMOFactory::createKomo() const
{
  auto komo = std::make_shared< ExtensibleKOMO >();
  for ( auto task : tasks_ )
  {
    komo->registerTask( task.first, task.second );
  }

  return komo;
}


//==============ExtensibleKOMO==============================================

ExtensibleKOMO::ExtensibleKOMO()
  : KOMO()
{

}

void ExtensibleKOMO::registerTask( const mlr::String & type, const SymbolGrounder & grounder )
{
  tasks_[ type ] = grounder;
}

void ExtensibleKOMO::groundTasks( double phase, const Graph& facts, int verbose )
{
  for(Node *n:facts)
  {
    if(!n->parents.N) continue; // skip not relevant node
    StringL symbols;
    for(Node *p:n->parents) symbols.append(&p->keys.last());// gather symbols

    if( n->keys.N && tasks_.count( n->keys.last() ) != 0 )
    {
      mlr::String type = n->keys.last();
      tasks_[ type ]( phase, facts, n, *this, verbose ); // ground the symbol
    }
    else if(n->keys.N && n->keys.last().startsWith("komo"))
    {
      HALT("UNKNOWN komo TAG: '" <<n->keys.last() <<"'");
    }
  }
}

//===========================================================================
ActionNode::ActionNode(mlr::KinematicWorld& kin, FOL_World& _fol, const KOMOFactory & komoFactory )
  : parent(NULL), s(0),
    fol(_fol), folState(NULL), folDecision(NULL), folReward(0.), folAddToState(NULL),
    startKinematics(kin), effKinematics(),
    rootMC(NULL), mcStats(NULL),
    komoPoseProblem(NULL), komoSeqProblem(NULL), komoPathProblem(NULL),
    mcCount(0), poseCount(0), seqCount(0), pathCount(0),
    symCost(0.), poseCost(0.), poseConstraints(0.), seqCost(0.), seqConstraints(0.), pathCost(0.), pathConstraints(0.),
    poseFeasible(false), seqFeasible(false), pathFeasible(false),
    inFringe1(false), inFringe2(false)
  , komoFactory_( komoFactory )
{
  //this is the root node!
  fol.reset_state();
  folState = fol.createStateCopy();
  rootMC = new PlainMC(fol);
  rootMC->verbose = 0;
}

ActionNode::ActionNode(ActionNode* parent, MCTS_Environment::Handle& a, const KOMOFactory & komoFactory )
  : parent(parent), fol(parent->fol),
     folState(NULL), folDecision(NULL), folReward(0.), folAddToState(NULL),
    startKinematics(parent->startKinematics), effKinematics(),
    rootMC(NULL), mcStats(NULL),
    komoPoseProblem(NULL), komoSeqProblem(NULL), komoPathProblem(NULL),
    mcCount(0), poseCount(0), seqCount(0), pathCount(0),
    symCost(0.), poseCost(0.), poseConstraints(0.), seqCost(0.), seqConstraints(0.), pathCost(0.), pathConstraints(0.),
    poseFeasible(false), seqFeasible(false), pathFeasible(false),
    inFringe1(false), inFringe2(false)
  , komoFactory_( komoFactory )
{
  s=parent->s+1;
  parent->children.append(this);
  fol.setState(parent->folState, parent->s);
  CHECK(a,"giving a 'NULL' shared pointer??");
  fol.transition(a);
  time=parent->time+fol.lastStepDuration;
  folReward = fol.lastStepReward;
  isTerminal = fol.successEnd;
  if(fol.deadEnd) isInfeasible=true;
  folState = fol.createStateCopy();
  folAddToState = NULL; //fresh creation -> notion to add
  folDecision = folState->getNode("decision");
  decision = a;
  rootMC = parent->rootMC;
}

void ActionNode::expand(){
  CHECK(!isExpanded,"");
  if(isTerminal) return;
  fol.setState(folState, s);
  auto actions = fol.get_actions();
  for(FOL_World::Handle& a:actions){
//    cout <<"  EXPAND DECISION: " <<*a <<endl;
    new ActionNode(this, a, komoFactory_);
  }
  if(!children.N) isTerminal=true;
  isExpanded=true;
}

arr ActionNode::generateRootMCRollouts(uint num, int stepAbort, const mlr::Array<MCTS_Environment::Handle>& prefixDecisions){
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

void ActionNode::addMCRollouts(uint num, int stepAbort){
  //-- collect decision path
  ActionNodeL treepath = getTreePath();
  mlr::Array<MCTS_Environment::Handle> prefixDecisions(treepath.N-1);
  for(uint i=1;i<treepath.N;i++)
    prefixDecisions(i-1) = treepath(i)->decision;

//  cout <<"DECISION PATH = "; listWrite(prefixDecisions); cout <<endl;

  arr R;
  for(uint k=0;k<num;k++){
    rootMC->initRollout(prefixDecisions);
    fol.setState(folState);
    double r = rootMC->finishRollout(stepAbort);
    R.append( r );
  }

  // the gathered rewards are inserted into the previous nodes
  for(ActionNode* n:treepath){
    if(!n->mcStats) n->mcStats = new MCStatistics;
    for(auto& r:R){
      n->mcStats->add(r);                  // is kept sorted!
      n->symCost = - n->mcStats->X.first();// take the best
    }
  }

  mcCount += num;
//  mcStats->report();
//  auto a = rootMC->getBestAction();
//  cout <<"******** BEST ACTION " <<*a <<endl;
}

void ActionNode::solvePoseProblem(){

  //reset the effective kinematics:
  if(parent && !parent->effKinematics.q.N){
    MLR_MSG("parent needs to have computed the pose first!");
    return;
  }
  if(!parent) effKinematics = startKinematics;
  else effKinematics = parent->effKinematics;

  //-- no need to collect 'path nodes'
  komoPoseProblem = komoFactory_.createKomo();
  komoPoseProblem->setModel(effKinematics);
  komoPoseProblem->setTiming(1., 2, 5., 1, false);

  komoPoseProblem->setHoming(-1., -1., 1e-1); //gradient bug??
  komoPoseProblem->setSquaredQVelocities();
//  komo.setSquaredFixJointVelocities(-1., -1., 1e3);
  komoPoseProblem->setSquaredFixSwitchedObjects(-1., -1., 1e3);

  //komoPoseProblem->setAbstractTask(0., *folState);
  komoPoseProblem->groundTasks( 0., *folState );
//  for(mlr::KinematicSwitch *sw: poseProblem->MP->switches){
//    sw->timeOfApplication=2;
//  }

  DEBUG( FILE("z.fol") <<fol; )
  DEBUG( komoPoseProblem->MP->reportFeatures(true, FILE("z.problem")); )
  komoPoseProblem->reset();
  try{
    komoPoseProblem->run();
  } catch(const char* msg){
    cout <<"KOMO FAILED: " <<msg <<endl;
  }
  COUNT_evals += komoPoseProblem->opt->newton.evals;
  COUNT_kin += mlr::KinematicWorld::setJointStateCount;
  COUNT_poseOpt++;
  poseCount++;

  DEBUG( komoPoseProblem->MP->reportFeatures(true, FILE("z.problem")); )

  Graph result = komoPoseProblem->getReport();
  DEBUG( FILE("z.problem.cost") <<result; )
  double cost = result.get<double>({"total","sqrCosts"});
  double constraints = result.get<double>({"total","constraints"});

  if(parent) cost += parent->poseCost;

  if(!pose.N || cost<poseCost){
    poseCost = cost;
    poseConstraints = constraints;
    poseFeasible = (constraints<.5);
    pose = komoPoseProblem->x;
  }

  if(!poseFeasible)
    labelInfeasible();

  effKinematics = *komoPoseProblem->MP->configurations.last();

  for(mlr::KinematicSwitch *sw: komoPoseProblem->MP->switches){
//    CHECK_EQ(sw->timeOfApplication, 1, "need to do this before the optimization..");
    if(sw->timeOfApplication>=2) sw->apply(effKinematics);
  }
  effKinematics.topSort();
  DEBUG( effKinematics.checkConsistency(); )
  effKinematics.getJointState();
}

void ActionNode::solveSeqProblem(int verbose){

  if(!s){ seqFeasible=true; return; } //there is no sequence to compute

  //-- collect 'path nodes'
  ActionNodeL treepath = getTreePath();

  komoSeqProblem = komoFactory_.createKomo();
  komoSeqProblem->setModel(startKinematics);
  komoSeqProblem->setTiming(time, 2, 5., 1, false);

  komoSeqProblem->setHoming(-1., -1., 1e-1); //gradient bug??
  komoSeqProblem->setSquaredQVelocities();
  komoSeqProblem->setSquaredFixJointVelocities(-1., -1., 1e3);
  komoSeqProblem->setSquaredFixSwitchedObjects(-1., -1., 1e3);

  for(ActionNode *node:treepath){
    komoSeqProblem->groundTasks((node->parent?node->parent->time:0.), *node->folState);//groundTasks((node->parent?node->parent->time:0.), *node->folState);
  }

  DEBUG( FILE("z.fol") <<fol; )
  DEBUG( komoSeqProblem->MP->reportFeatures(true, FILE("z.problem")); )
  komoSeqProblem->reset();
  try{
    komoSeqProblem->run();
  } catch(const char* msg){
    cout <<"KOMO FAILED: " <<msg <<endl;
  }
  COUNT_evals += komoSeqProblem->opt->newton.evals;
  COUNT_kin += mlr::KinematicWorld::setJointStateCount;
  COUNT_seqOpt++;
  seqCount++;

  DEBUG( komoSeqProblem->MP->reportFeatures(true, FILE("z.problem")); )
//  komo.checkGradients();

  Graph result = komoSeqProblem->getReport();
  DEBUG( FILE("z.problem.cost") <<result; )
  double cost = result.get<double>({"total","sqrCosts"});
  double constraints = result.get<double>({"total","constraints"});

  if(!seq.N || cost<seqCost){
    seqCost = cost;
    seqConstraints = constraints;
    seqFeasible = (constraints<.5);
    seq = komoSeqProblem->x;
  }

  if(!seqFeasible)
    labelInfeasible();
}

void ActionNode::solvePathProblem(uint microSteps, int verbose){

  if(!s){ pathFeasible=true; return; } //there is no path to compute

  //-- collect 'path nodes'
  ActionNodeL treepath = getTreePath();

  komoPathProblem = komoFactory_.createKomo();
  komoPathProblem->setModel(startKinematics);
  komoPathProblem->setTiming(time, microSteps, 5., 2, false);

  komoPathProblem->setHoming(-1., -1., 1e-2); //gradient bug??
  komoPathProblem->setSquaredQAccelerations();
  komoPathProblem->setSquaredFixJointVelocities(-1., -1., 1e3);
  komoPathProblem->setSquaredFixSwitchedObjects(-1., -1., 1e3);

  for(ActionNode *node:treepath){
    komoPathProblem->groundTasks((node->parent?node->parent->time:0.), *node->folState);
  }

  DEBUG( FILE("z.fol") <<fol; )
  DEBUG( komoPathProblem->MP->reportFeatures(true, FILE("z.problem")); )
  komoPathProblem->reset();
  try{
    komoPathProblem->run();
  } catch(const char* msg){
    cout <<"KOMO FAILED: " <<msg <<endl;
  }
  COUNT_evals += komoPathProblem->opt->newton.evals;
  COUNT_kin += mlr::KinematicWorld::setJointStateCount;
  COUNT_pathOpt++;
  pathCount++;

  DEBUG( komoPathProblem->MP->reportFeatures(true, FILE("z.problem")); )
//  komo.checkGradients();

  Graph result = komoPathProblem->getReport();
  DEBUG( FILE("z.problem.cost") <<result; )
  double cost = result.get<double>({"total","sqrCosts"});
  double constraints = result.get<double>({"total","constraints"});

  if(!path.N || cost<pathCost){
    pathCost = cost;
    pathConstraints = constraints;
    pathFeasible = (constraints<.5);
    path = komoPathProblem->x;
  }

  if(!pathFeasible)
    labelInfeasible();
}

void ActionNode::labelInfeasible(){
  isInfeasible = true;

  //-- remove children
//  ActionNodeL tree;
//  getAllChildren(tree);
//  for(ActionNode *n:tree) if(n!=this) delete n; //TODO: memory leak!
  DEL_INFEASIBLE( children.clear(); )

  //-- add INFEASIBLE flag to fol
  NodeL symbols = folDecision->parents;
  symbols.prepend( fol.KB.getNode({"INFEASIBLE"}));

//  cout <<"\n *** LABELLING INFEASIBLE: "; listWrite(symbols); cout <<endl;
  //-- find the right parent...
  ActionNode* node = this;
  while(node->parent){
    bool stop=false;
    for(Node *fact:node->folState->list()){
      if(fact->keys.N && fact->keys.last()=="block"){
        if(tuplesAreEqual(fact->parents, symbols)){
          CHECK(fact->isOfType<bool>() && fact->keys.first()=="block", "");
          stop=true;
          break;
        }
      }
    }
    if(stop) break;
    node = node->parent;
  }

  if(!node->folAddToState){
    node->folAddToState = &fol.KB.newSubgraph({"ADD"}, {node->folState->isNodeOfGraph})->value;
  }
  node->folAddToState->newNode<bool>({}, symbols, true);

//  ActionNode *root=getRoot();
  node->recomputeAllFolStates();
  node->recomputeAllMCStats(false);

}

ActionNodeL ActionNode::getTreePath(){
  ActionNodeL path;
  ActionNode *node=this;
  for(;node;){
    path.prepend(node);
    node = node->parent;
  }
  return path;
}

ActionNode* ActionNode::getRoot(){
  ActionNode* n=this;
  while(n->parent) n=n->parent;
  return n;
}

void ActionNode::getAllChildren(ActionNodeL& tree){
  for(ActionNode* c:children) c->getAllChildren(tree);
  tree.append(this);
}

ActionNode *ActionNode::treePolicy_random(){
  if(isInfeasible) return NULL;
  if(isTerminal) return NULL;
  if(children.N) return children.rndElem()->treePolicy_random();
  return this;
}

ActionNode *ActionNode::treePolicy_softMax(double temperature){
  if(isInfeasible) return NULL;
  if(isTerminal){
//    LOG(0) <<"stuck at terminal:" <<*this <<endl;
    return NULL;
  }
  if(children.N){
    arr Q(children.N);
    for(uint i=0;i<children.N;i++) Q(i) = children.elem(i)->symCost;
//    rndGauss(Q, .1, true);
    Q *= -temperature;
    Q = exp(Q);
    normalizeDist(Q);
    uint best = sampleMultinomial(Q); //.maxIndex();
    return children.elem(best)->treePolicy_softMax(temperature);
  }
  return this;
}

bool ActionNode::recomputeAllFolStates(){
  if(!parent){ //this is root
    folState->copy(*fol.start_state);
    if(folAddToState) applyEffectLiterals(*folState, *folAddToState, {}, NULL);
  }else{
    fol.setState(parent->folState, parent->s);
    if(fol.is_feasible_action(decision)){
      fol.transition(decision);
      time=parent->time+fol.lastStepDuration;
      folReward = fol.lastStepReward;
      isTerminal = fol.successEnd;
      if(fol.deadEnd){
        if(!seqFeasible && !pathFeasible) //seq or path have already proven it feasible! Despite the logic...
          isInfeasible=true;
        return false;
      }
      folState->copy(*fol.state);
      if(folAddToState) applyEffectLiterals(*folState, *folAddToState, {}, NULL);
      folDecision = folState->getNode("decision");
    }else{
      if(!seqFeasible && !pathFeasible) //seq or path have already proven it feasible! Despite the logic...
        isInfeasible=true;
      return false;
    }
  }
  if(children.N){
    for(uint i=children.N-1;;){
      bool feasible = children(i)->recomputeAllFolStates();
      DEL_INFEASIBLE( if(!feasible) children.remove(i); )
      if(!i || !children.N) break;
      i--;
    }
  }
  DEBUG( if(!parent) FILE("z.fol") <<fol; )
  return true;
}

void ActionNode::recomputeAllMCStats(bool excludeLeafs){
  if(!mcStats) return;
  if(!isTerminal){
    if(children.N || !excludeLeafs || isInfeasible)
      mcStats->clear();
  }
  for(ActionNode* ch:children){
    ch->recomputeAllMCStats(excludeLeafs);
    for(double x:ch->mcStats->X) mcStats->add(x);
  }
  if(mcStats->n)
    symCost = - mcStats->X.first();
  else
    symCost = 100.;
}

void ActionNode::checkConsistency(){
  //-- check that the state->parent points to the parent's state
  if(parent){
    CHECK_EQ(parent->folState->isNodeOfGraph, folState->isNodeOfGraph->parents.scalar(), "");
    CHECK_EQ(&folDecision->container, folState, "");
  }

  //-- check that each child exactly matches a decision, in same order
  if(children.N){
    fol.setState(folState, s);
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

void ActionNode::write(ostream& os, bool recursive) const{
  os <<"------- NODE -------\ns=" <<s <<" t=" <<time;
  if(decision) os <<" a= " <<*decision <<endl;
  else os <<" a=<ROOT>"<<endl;

  for(uint i=0;i<s+1;i++) os <<"  ";
  os <<" state= " <<*folState->isNodeOfGraph <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" depth=" <<s <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" poseCost=" <<poseCost <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" seqCost=" <<seqCost <<endl;
  for(uint i=0;i<s+1;i++) os <<"  ";  os <<" pathCost=" <<pathCost <<endl;
  if(recursive) for(ActionNode *n:children) n->write(os);
}

void ActionNode::getGraph(Graph& G, Node* n) {
  if(!n){
    n = G.newNode<bool>({"a:<ROOT>"}, NodeL(), true);
  }else{
    n = G.newNode<bool>({STRING("a:"<<*decision)}, {n}, true);
  }
  graphIndex = n->index;
  n->keys.append(STRING("s:" <<s <<" t:" <<time <<' ' <<folState->isNodeOfGraph->keys.scalar()));
  if(mcStats && mcStats->n) n->keys.append(STRING("MC best:" <<mcStats->X.first() <<" n:" <<mcStats->n));
  n->keys.append(STRING("sym  #" <<mcCount <<" f:" <<symCost <<" terminal:" <<isTerminal));
  n->keys.append(STRING("pose #" <<poseCount <<" f:" <<poseCost <<" g:" <<poseConstraints <<" feasible:" <<poseFeasible));
  n->keys.append(STRING("seq  #" <<seqCount <<" f:" <<seqCost <<" g:" <<seqConstraints <<" feasible:" <<seqFeasible));
  n->keys.append(STRING("path #" <<pathCount <<" f:" <<pathCost <<" g:" <<pathConstraints <<" feasible:" <<pathFeasible));
  if(folAddToState) n->keys.append(STRING("symAdd:" <<*folAddToState));

  G.getRenderingInfo(n).dotstyle="shape=box";
  if(isInfeasible){
    if(isTerminal)  G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=violet";
    else G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=red";
  }else if(isTerminal){
    if(seqCount || pathCount) G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=cyan";
    else G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=blue";
  }else{
    if(poseCount || seqCount || pathCount) G.getRenderingInfo(n).dotstyle <<" style=filled fillcolor=green";
  }
//  if(inFringe1) G.getRenderingInfo(n).dotstyle <<" color=green";
  if(inFringe1) G.getRenderingInfo(n).dotstyle <<" peripheries=2";
  if(inFringe2) G.getRenderingInfo(n).dotstyle <<" peripheries=3";

//  n->keys.append(STRING("reward:" <<effPoseReward));
  for(ActionNode *ch:children) ch->getGraph(G, n);
}

void ActionNode::getAll(ActionNodeL& L){
  L.append(this);
  for(ActionNode *ch:children) ch->getAll(L);
}

RUN_ON_INIT_BEGIN(manipulationTree)
ActionNodeL::memMove = true;
RUN_ON_INIT_END(manipulationTree)
