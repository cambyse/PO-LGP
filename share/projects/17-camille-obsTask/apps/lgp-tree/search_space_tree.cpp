#include "search_space_tree.h"

#include <Core/util.tpp>

#include <Motion/komo.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>

#include <LGP/LGP.h>

SearchSpaceTree::SearchSpaceTree( const KOMOFactory & komoFactory )
  : poseView("pose", 1., -0)
  , seqView("sequence", 1., -0)
  , pathView("path", .1, -1)
  , komoFactory_(komoFactory)
{
//  poseView.writeToFiles = false;
//  seqView.writeToFiles = false;
//  pathView.writeToFiles = false;
}

void SearchSpaceTree::prepareKin( const std::string & kinDescription ){
  kin.init( kinDescription.c_str() );
  //  kin.watch();
  computeMeshNormals(kin.shapes);

  kin.calc_fwdPropagateFrames();
//  kin.watch(/*true*/);
}

void SearchSpaceTree::prepareFol( const std::string & folDescription ){
  // get number of possible worlds
  Graph KB;
  KB.read(FILE(folDescription.c_str()));
  auto bsGraph = &KB.get<Graph>("BELIEF_START_STATE");
  uint nWorlds = bsGraph->d0;

  // generate all the possible fol
  folWorlds_ = mlr::Array< std::shared_ptr<FOL_World> > ( nWorlds );
  bs_ = arr( nWorlds );
  for( uint w = 0; w < nWorlds; w++ )
  {
    // retrieve the facts of the belief state
    std::shared_ptr<FOL_World> fol = std::make_shared<FOL_World>();
    fol->init(FILE(folDescription.c_str()));
    auto n = bsGraph->elem(w);
    StringA fact;
    // add fact
    for( auto s : n->parents ) fact.append( s->keys.last() );//std::cout << n->parents.last()->keys << std::endl;
    fol->addFact(fact);

    // tag this fact as not observable
    StringA notObservableFact; notObservableFact.append( "NOT_OBSERVABLE" );
    for( auto s : fact ) notObservableFact.append( s );

    fol->addFact(notObservableFact);
    fol->reset_state();

    //std::cout << *fol << std::endl; // tmp
    folWorlds_(w) = fol;
    bs_(w) = n->get<double>();
  }

  fol.init(FILE(folDescription.c_str()));
  fol.reset_state();
}

void SearchSpaceTree::prepareTree(){
  root = new ActionNode(kin, fol, folWorlds_, bs_, komoFactory_);
  node = root;
}

void SearchSpaceTree::prepareDisplay(){
  threadOpenModules(true);
}

void SearchSpaceTree::updateDisplay(){
  if(node->komoPoseProblem && node->komoPoseProblem->MP->configurations.N)
    poseView.setConfigurations(node->komoPoseProblem->MP->configurations);
  if(node->komoSeqProblem && node->komoSeqProblem->MP->configurations.N)
    seqView.setConfigurations(node->komoSeqProblem->MP->configurations);
  else seqView.clear();
  if(node->komoPathProblem && node->komoPathProblem->MP->configurations.N)
    pathView.setConfigurations(node->komoPathProblem->MP->configurations);
  else pathView.clear();


  ActionNodeL all = root->getAll();
  for(auto& n:all) n->inFringe1=n->inFringe2=false;
  for(auto& n:poseFringe) n->inFringe1=true;
  //  for(auto& n:seqFringe) n->inFringe1=true;
  for(auto& n:mcFringe) n->inFringe2=true;

  Graph dot=root->getGraph();
  dot.writeDot(FILE("z.dot"), false, false, 0, node->graphIndex);
  int r = system("dot -Tpdf z.dot > z.pdf");
  if(r) LOG(-1) <<"could not startup dot";
}

bool SearchSpaceTree::execRandomChoice(){
  mlr::String cmd;
  if(rnd.uni()<.5){
    switch(rnd.num(5)){
      case 0: cmd="u"; break;
      case 1: cmd="p"; break;
      case 2: cmd="s"; break;
      case 3: cmd="x"; break;
      case 4: cmd="m"; break;
    }
  }else{
    cmd <<rnd(node->children.N);
  }
  return execChoice(cmd);
}

bool SearchSpaceTree::execChoice( mlr::String cmd ){
  cout <<"COMMAND: '" <<cmd <<"'" <<endl;

  if(cmd=="q") return false;
  else if(cmd=="u"){ if(node->parent) node = node->parent; }
  else if(cmd=="p") node->solvePoseProblem();
  else if(cmd=="s") node->solveSeqProblem();
  else if(cmd=="x") node->solvePathProblem(20);
  else if(cmd=="m") node->addMCRollouts(100,10);
  else{
    int choice;
    cmd >> choice;
    cout <<"CHOICE=" <<choice <<endl;
    if(choice>=(int)node->children.N){
      cout <<"--- there is no such choice" <<endl;
    }else{
      node = node->children(choice);
      if(!node->isExpanded){
        node->expand();
        if(autoCompute){
          node->solvePoseProblem();
          //          node->solveSeqProblem();
          //          node->solvePathProblem(20);
        }
      }
    }
  }
  return true;
}

void SearchSpaceTree::printChoices() const{
  //-- query UI
  cout <<"********************" <<endl;
  cout <<"NODE:\n" <<*node <<endl;
  cout <<"--------------------" <<endl;
  cout <<"\nCHOICES:" <<endl;
  cout <<"(q) quit" <<endl;
  cout <<"(u) up" <<endl;
  cout <<"(p) pose problem" <<endl;
  cout <<"(s) sequence problem" <<endl;
  cout <<"(x) path problem" <<endl;
  cout <<"(m) MC planning" <<endl;
  uint c=0;
  for(ActionNode* a:node->children){
    cout <<"(" <<c++ <<") DECISION: " <<*a->decision <<endl;
  }
}

mlr::String SearchSpaceTree::queryForChoice() const{
  mlr::String cmd;
  std::string tmp;
  getline(std::cin, tmp);
  cmd=tmp.c_str();
  return cmd;
}

//===========================================================================

double poseHeuristic(MNode* n){
  return n->symCost;
}

double mcHeuristic(MNode* n){
  if(n->poseCount) return -10.+n->poseCost;
  return 1.;
}

double seqHeuristic(MNode* n){
  return n->symCost;
}

double poseCost(MNode* n){
  if(!n->poseCount || !n->poseFeasible) return 100.;
  return .1*n->symCost+n->poseCost;
}

double seqCost(MNode* n){
  if(!n->seqCount || !n->seqFeasible) return 100.;
  return .1*n->symCost+n->seqCost;
}

double pathHeuristic(MNode* n){
  return seqCost(n);
}

double pathCost(MNode* n){
  if(!n->path.N || !n->pathFeasible) return 100.;
  return .1*n->symCost + n->seqCost + n->pathCost;
}

MNode* getBest(mlr::Array<MNode*>& fringe, double heuristic(MNode*)){
  if(!fringe.N) return NULL;
  MNode* best=NULL;
  for(MNode* n:fringe)
    if(!best || heuristic(n)<heuristic(best)) best=n;
  return best;
}

MNode* popBest(mlr::Array<MNode*>& fringe, double heuristic(MNode*)){
  if(!fringe.N) return NULL;
  MNode* best=getBest(fringe, heuristic);
  fringe.removeValue(best);
  return best;
}
