#include "ao_search.h"

#include <list>

#include <Core/util.tpp>

#include <Motion/komo.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>

#include <LGP/LGP.h>

//=====================free functions======================
static double eps() { return std::numeric_limits< double >::epsilon(); }

//===========================================================================
AOSearch::AOSearch( const KOMOFactory & komoFactory )
  : komoFactory_( komoFactory )
//  , poseView_( "pose" , 1., -0   )
//  , seqView_ ("sequence", 1., -0 )
//  , pathView_( "path", .1, -1    )
//  , pathView2_( "path2", .1, -1  )
{

}

// modifiers
void AOSearch::prepareFol( const std::string & folDescription )
{
  const mlr::String notObservableTag = "NOT_OBSERVABLE";

  Graph KB;
  KB.read( FILE( folDescription.c_str() ) );

  // fully observable case
  if( KB[ beliefStateTag_ ] == nullptr )
  {
    // create dummy array
    folWorlds_ = mlr::Array< std::shared_ptr<FOL_World> > ( 1 );
    std::shared_ptr<FOL_World> fol = std::make_shared<FOL_World>();
    fol->init(FILE(folDescription.c_str()));
    folWorlds_( 0 ) = fol;
    fol->reset_state();
    // create dummy bs in observable case
    bs_ = arr( 1 );
    bs_( 0 ) = 1.0;
  }
  // partially observable case
  else
  {
    // get number of possible worlds
    auto bsGraph = &KB.get<Graph>( beliefStateTag_ );
    const uint nWorlds = bsGraph->d0;

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
      for( auto s : n->parents ) fact.append( s->keys.last() );
      //fol->addFact(fact);

      // tag this fact as not observable
      StringA notObservableFact; notObservableFact.append( notObservableTag );
      for( auto s : fact ) notObservableFact.append( s );

      fol->addFact(notObservableFact);
      fol->reset_state();

      //std::cout << *fol << std::endl; // tmp
      folWorlds_(w) = fol;
      bs_(w) = n->get<double>();
    }

    // check that the belief state sums to 1
    double total = 0;
    for( auto p : bs_ ) total += p;

    CHECK( total == 1.00, "wrong belief state definition, the total of the probabilities doesn't sum to 1" );
  }
}

void AOSearch::prepareKin( const std::string & kinDescription )
{
  Graph G( kinDescription.c_str() );

  if( G[ beliefStateTag_ ] == nullptr )
  {
    auto kin = std::make_shared< mlr::KinematicWorld >();
    kin->init( kinDescription.c_str() );
    computeMeshNormals( kin->shapes );
    kin->calc_fwdPropagateFrames();
    //kin->watch(/*true*/);

    kinematics_.append( kin );
  }
  else
  {
    auto bsGraph = &G.get<Graph>( beliefStateTag_ );
    const uint nWorlds = bsGraph->d0;

    // build the different worlds
    for( uint w = 0; w < nWorlds; w++ )
    {
      Graph kinG( kinDescription.c_str() );

      // copy unobservable facts
      auto n = bsGraph->elem(w);

      for( auto nn : n->graph() )
      {
        nn->newClone( kinG );
      }

      auto kin = std::make_shared< mlr::KinematicWorld >();
      kin->init( kinG );
      computeMeshNormals( kin->shapes );
      kin->calc_fwdPropagateFrames();
      //
      //kin->watch(/*true*/);
      //
      kinematics_.append( kin );
    }
  }
}

void AOSearch::prepareTree()
{
  CHECK( folWorlds_.d0 == kinematics_.d0, "There should be as many logic worlds as kinematic worlds!, check the fol and kin description files!" );

  root_ = new AONode( folWorlds_, kinematics_, bs_, komoFactory_ );
}

void AOSearch::prepareDisplay()
{
  for( auto w = 0; w < folWorlds_.d0; ++w )
  {
    std::string namePose = std::string( "pose" ) + std::string( "-world-" ) + std::to_string( w );
    std::string nameSeq =  std::string( "seq" ) + std::string( "-world-" ) + std::to_string( w );
    std::string namePath = std::string( "path" ) + std::string( "-world-" ) + std::to_string( w );

    poseViews_.append( std::make_shared< OrsPathViewer >( namePose.c_str(),  1, -0   ) );
    seqViews_.append( std::make_shared< OrsPathViewer >( nameSeq.c_str(),    1, -0   ) );
    pathViews_.append( std::make_shared< OrsPathViewer >( namePath.c_str(), .1, -1   ) );
  }

  threadOpenModules( true );
}

void AOSearch::optimizePoses()
{
  optimizePoses( root_ );
}

void AOSearch::optimizeSequences()
{
  auto nodes = getTerminalNodes();

  for( auto n : nodes )
  {
    n->solveSeqProblem();
  }
}

void AOSearch::optimizePaths()
{
  auto nodes = getTerminalNodes();

  for( auto n : nodes )
  {
    n->solvePathProblem( 20 );
  }
}

void AOSearch::optimizeJointPaths()
{
  auto nodes = getTerminalNodes();

  for( auto n : nodes )
  {
    n->solveJointPathProblem( 20 );
  }
}

/*void AOSearch::optimizePaths2()
{
  optimizePaths2( root_, root_ );
}*/

/*void AOSearch::optimizePaths2( AONode * node, AONode * start )
{
  if( node->isTerminal() )
  {
    node->solvePathProblem2( 20, start );
  }
  else
  {
    if( node->andSiblings().d0 >= 1 )
    {
      node->solvePathProblem2( 20, start );
      start = node;
    }

    for( auto c : node->bestFamily() )
    {
      optimizePaths2( c, start );
    }
  }
}*/

void AOSearch::optimizePoses( AONode * node )
{
  node->solvePoseProblem();

  for( auto c : node->bestFamily() )
  {
    optimizePoses( c );
  }
}

void AOSearch::updateDisplay( const WorldID & ww, bool poses, bool seqs, bool paths )
{
  std::list< std::size_t > worldIds;

  if( ww.id() == -1 )
  {
    for ( auto w = 0; w < folWorlds_.d0; ++w )
    {
      worldIds.push_back( w );
    }
  }
  else
  {
    worldIds.push_back( ww.id() );
  }

  for( auto w : worldIds )
  {
    // get the terminal node for the world w, in the case of stochaticity
    AONode * node = getTerminalNode( WorldID( w ) );

    if( poses && node->komoPoseProblems()( w ) && node->komoPoseProblems()( w )->MP->configurations.N )
      poseViews_( w )->setConfigurations( node->komoPoseProblems()( w )->MP->configurations );
    else poseViews_( w )->clear();

    if( seqs && node->komoSeqProblems()( w ) && node->komoSeqProblems()( w )->MP->configurations.N )
      seqViews_( w )->setConfigurations( node->komoSeqProblems()( w )->MP->configurations );
    else seqViews_( w )->clear();

//    if( paths && node->komoPathProblems()( w ) && node->komoPathProblems()( w )->MP->configurations.N )
//      pathViews_( w )->setConfigurations( node->komoPathProblems()( w )->MP->configurations );
//    else pathViews_( w )->clear();

    if( paths && node->komoJointPathProblems()( w ) && node->komoJointPathProblems()( w )->MP->configurations.N )
      pathViews_( w )->setConfigurations( node->komoJointPathProblems()( w )->MP->configurations );
    else pathViews_( w )->clear();
  }
}

mlr::Array< AONode * > AOSearch::getNodesToExpand() const
{
  return getNodesToExpand( root_ );
}

mlr::Array< AONode * > AOSearch::getNodesToExpand( AONode * node ) const
{
  mlr::Array< AONode * >  nodes;
  // starts from root
  if( ! node->isSolved() )
  {
    if( ! node->isExpanded() )
      nodes.append( node );
    else
    {
      for( auto c : node->bestFamily() )
      {
        if( ! c->isExpanded() )
          nodes.append( c );
        else
          nodes.append( getNodesToExpand( c ) );
      }
    }
  }

  return nodes;
}

mlr::Array< AONode * > AOSearch::getTerminalNodes() const
{
  return getTerminalNodes( root_ );
}

mlr::Array< AONode * > AOSearch::getTerminalNodes( AONode * n ) const
{
  mlr::Array< AONode * > nodes;

  if( n->isTerminal() )
  {
    nodes.append( n );
  }
  else
  {
    for( auto c : n->bestFamily() )
    {
      nodes.append( getTerminalNodes( c ) );
    }
  }

  return nodes;
}

AONode * AOSearch::getTerminalNode( const WorldID & w ) const
{
  // could be more generale and return a list of node in case of stochastic world
  return getTerminalNode( root_, w );
}

AONode * AOSearch::getTerminalNode( AONode * n, const WorldID & w ) const
{
  AONode * node = nullptr;
  if( n->isTerminal() )
  {
    CHECK( n->bs()( w.id() ) > eps(), "bug in getTerminalNode function, the belief state of the found node is invalid!" );
    node = n;
  }
  else
  {
    for( auto c : n->bestFamily() )
    {
      if( c->bs()( w.id() ) > eps() )
      {
        node = getTerminalNode( c, w );
        break;
      }
    }
  }

  return node;
}

void AOSearch::printPolicy( std::iostream & ss ) const
{
  ss << "digraph g{" << std::endl;

  printPolicy( root_, ss );

  ss << "}" << std::endl;
}

void AOSearch::printPolicy( AONode * node, std::iostream & ss ) const
{
  for( auto c : node->bestFamily() )
  {
    std::stringstream ss1;
    ss1 << node->bestActionStr();

    auto diffFacts = c->differentiatingFacts();

    for( auto fact : c->differentiatingFacts() )
      ss1 << std::endl << fact;

    if( node->bestFamily().N > 1 )
    {
      ss1 << std::endl << "p=" << c->pHistory();
      ss1 << std::endl << "q=" << c->pHistory() / node->pHistory();
    }

    auto label = ss1.str();

    ss << node->id() << "->" << c->id() << " [ label=\"" << label << "\" ]" << ";" << std::endl;

    printPolicy( c, ss );
  }
}
//===========================================================================

//SearchSpaceTree::SearchSpaceTree( const KOMOFactory & komoFactory )
//  : poseView("pose", 1., -0)
//  , seqView("sequence", 1., -0)
//  , pathView("path", .1, -1)
//  , komoFactory_(komoFactory)
//{
////  poseView.writeToFiles = false;
////  seqView.writeToFiles = false;
////  pathView.writeToFiles = false;
//}

//void SearchSpaceTree::prepareKin( const std::string & kinDescription ){
//  kin.init( kinDescription.c_str() );
//  //  kin.watch();
//  computeMeshNormals(kin.shapes);

//  kin.calc_fwdPropagateFrames();
////  kin.watch(/*true*/);
//}

//void SearchSpaceTree::prepareFol( const std::string & folDescription ){

//  // get number of possible worlds
//  Graph KB;
//  KB.read(FILE(folDescription.c_str()));
//  auto bsGraph = &KB.get<Graph>("BELIEF_START_STATE");
//  uint nWorlds = bsGraph->d0;

//  // generate all the possible fol
//  folWorlds_ = mlr::Array< std::shared_ptr<FOL_World> > ( nWorlds );
//  bs_ = arr( nWorlds );
//  for( uint w = 0; w < nWorlds; w++ )
//  {
//    // retrieve the facts of the belief state
//    std::shared_ptr<FOL_World> fol = std::make_shared<FOL_World>();
//    fol->init(FILE(folDescription.c_str()));
//    auto n = bsGraph->elem(w);
//    StringA fact;
//    // add fact
//    for( auto s : n->parents ) fact.append( s->keys.last() );
//    fol->addFact(fact);

//    // tag this fact as not observable
//    StringA notObservableFact; notObservableFact.append( "NOT_OBSERVABLE" );
//    for( auto s : fact ) notObservableFact.append( s );

//    fol->addFact(notObservableFact);
//    fol->reset_state();

//    //std::cout << *fol << std::endl; // tmp
//    folWorlds_(w) = fol;
//    bs_(w) = n->get<double>();
//  }
//}

//void SearchSpaceTree::prepareTree(){
//  //root = new ActionNode(kin, fol, folWorlds_, bs_, komoFactory_);
//  root = new PartiallyObservableNode(kin, folWorlds_, komoFactory_);
//  node = root;
//}


//void SearchSpaceTree::prepareDisplay(){
//  threadOpenModules(true);
//}

//void SearchSpaceTree::updateDisplay(){
//  if(node->komoPoseProblem() && node->komoPoseProblem()->MP->configurations.N)
//    poseView.setConfigurations(node->komoPoseProblem()->MP->configurations);
//  if(node->komoSeqProblem() && node->komoSeqProblem()->MP->configurations.N)
//    seqView.setConfigurations(node->komoSeqProblem()->MP->configurations);
//  else seqView.clear();
//  if(node->komoPathProblem() && node->komoPathProblem()->MP->configurations.N)
//    pathView.setConfigurations(node->komoPathProblem()->MP->configurations);
//  else pathView.clear();


//  PartiallyObservableNodeL all = root->getAll();
//  for(auto& n:all) n->inFringe1()=n->inFringe2()=false;
//  for(auto& n:poseFringe()) n->inFringe1()=true;
//  //  for(auto& n:seqFringe) n->inFringe1=true;
//  for(auto& n:mcFringe) n->inFringe2()=true;

//  Graph dot=root->getGraph();
//  dot.writeDot(FILE("z.dot"), false, false, 0, node->graphIndex());
//  int r = system("dot -Tpdf z.dot > z.pdf");
//  if(r) LOG(-1) <<"could not startup dot";
//}

//bool SearchSpaceTree::execRandomChoice(){
//  mlr::String cmd;
//  if(rnd.uni()<.5){
//    switch(rnd.num(5)){
//      case 0: cmd="u"; break;
//      case 1: cmd="p"; break;
//      case 2: cmd="s"; break;
//      case 3: cmd="x"; break;
//      case 4: cmd="m"; break;
//    }
//  }else{
//    cmd <<rnd(node->children().N);
//  }
//  return execChoice(cmd);
//}

//bool SearchSpaceTree::execChoice( mlr::String cmd ){
//  cout <<"COMMAND: '" <<cmd <<"'" <<endl;

//  if(cmd=="q") return false;
//  else if(cmd=="u"){ if(node->parent()) node = node->parent(); }
//  else if(cmd=="p") node->solvePoseProblem();
//  else if(cmd=="s") node->solveSeqProblem();
//  else if(cmd=="x") node->solvePathProblem(20);
//  else if(cmd=="m") node->addMCRollouts(100,10);
//  else{
//    int choice;
//    cmd >> choice;
//    cout <<"CHOICE=" <<choice <<endl;
//    if(choice>=(int)node->children().N){
//      cout <<"--- there is no such choice" <<endl;
//    }else{
//      node = node->children()(choice);
//      if(!node->isExpanded()){
//        node->expand();
//        if(autoCompute){
//          node->solvePoseProblem();
//          //          node->solveSeqProblem();
//          //          node->solvePathProblem(20);
//        }
//      }
//    }
//  }
//  return true;
//}

//void SearchSpaceTree::printChoices() const{
//  //-- query UI
//  cout <<"********************" <<endl;
//  cout <<"NODE:\n" <<*node <<endl;
//  cout <<"--------------------" <<endl;
//  cout <<"\nCHOICES:" <<endl;
//  cout <<"(q) quit" <<endl;
//  cout <<"(u) up" <<endl;
//  cout <<"(p) pose problem" <<endl;
//  cout <<"(s) sequence problem" <<endl;
//  cout <<"(x) path problem" <<endl;
//  cout <<"(m) MC planning" <<endl;
//  uint c=0;
//  for(PartiallyObservableNode* a:node->children()){
//    cout <<"(" <<c++ <<") DECISION: " <<*a->decision() <<endl;
//  }
//}

//mlr::String SearchSpaceTree::queryForChoice() const{
//  mlr::String cmd;
//  std::string tmp;
//  getline(std::cin, tmp);
//  cmd=tmp.c_str();
//  return cmd;
//}

////===========================================================================

//double poseHeuristic(PartiallyObservableNode* n){
//  return n->symCost();
//}

//double mcHeuristic(PartiallyObservableNode* n){
//  if(n->poseCount()) return -10.+n->poseCost();
//  return 1.;
//}

//double seqHeuristic(PartiallyObservableNode* n){
//  return n->symCost();
//}

//double poseCost(PartiallyObservableNode* n){
//  if(!n->poseCount() || !n->poseFeasible()) return 100.;
//  return .1*n->symCost()+n->poseCost();
//}

//double seqCost(PartiallyObservableNode* n){
//  if(!n->seqCount() || !n->seqFeasible()) return 100.;
//  return .1*n->symCost()+n->seqCost();
//}

//double pathHeuristic(PartiallyObservableNode* n){
//  return seqCost(n);
//}

//double pathCost(PartiallyObservableNode* n){
//  if(!n->path().N || !n->pathFeasible()) return 100.;
//  return .1*n->symCost() + n->seqCost() + n->pathCost();
//}

//PartiallyObservableNode* getBest(mlr::Array<PartiallyObservableNode*>& fringe, double heuristic(PartiallyObservableNode*)){
//  if(!fringe.N) return NULL;
//  PartiallyObservableNode* best=NULL;
//  for(PartiallyObservableNode* n:fringe)
//    if(!best || heuristic(n)<heuristic(best)) best=n;
//  return best;
//}

//PartiallyObservableNode* popBest(mlr::Array<PartiallyObservableNode*>& fringe, double heuristic(PartiallyObservableNode*)){
//  if(!fringe.N) return NULL;
//  PartiallyObservableNode* best=getBest(fringe, heuristic);
//  fringe.removeValue(best);
//  return best;
//}

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
