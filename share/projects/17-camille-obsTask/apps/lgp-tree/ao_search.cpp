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

void AOSearch::solveSymbolically()
{
  auto s = 0;
  while( ! isSymbolicallySolved() )
  {
    s++;
    auto nodes = getNodesToExpand();

    for( auto node : nodes )
    {
      // expand
      node->expand();

      // generate rollouts for each child
      for( auto f : node->families() )
      {
        for( auto c : f )
        {
          c->generateMCRollouts( 50, 10 );
        }
      }

      // backtrack result
      node->backTrackBestExpectedPolicy();
    }
  }
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
  if( ! node->isSymbolicallySolved() )
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

  if( n->isSymbolicallyTerminal() )
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
  if( n->isSymbolicallyTerminal() )
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
