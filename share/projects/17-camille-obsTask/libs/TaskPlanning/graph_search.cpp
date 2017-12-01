/*  ------------------------------------------------------------------
    Copyright 2016 Camille Phiquepal
    email: camille.phiquepal@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include <graph_search.h>
#include <policy_builder.h>
#include <graph_printer.h>

#include <chrono>
#include <functional>
#include <queue>

//=====================free functions======================
static double eps() { return std::numeric_limits< double >::epsilon(); }
static double m_inf() { return -std::numeric_limits< double >::max(); }

namespace tp
{

void GraphSearchPlanner::setFol( const std::string & folDescription )
{
  const mlr::String notObservableTag = "NOT_OBSERVABLE";

  Graph KB;
  KB.read( FILE( folDescription.c_str() ) );
  //KB.isDoubleLinked = false;
  // fully observable case
  if( KB[ beliefStateTag_ ] == nullptr )
  {
    // create dummy array
    folEngines_ = mlr::Array< std::shared_ptr<FOL_World> > ( 1 );
    std::shared_ptr<FOL_World> fol = std::make_shared<FOL_World>();
    fol->init(FILE(folDescription.c_str()));
    folEngines_( 0 ) = fol;
    fol->reset_state();
    //fol->KB.isDoubleLinked = false;
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
    folEngines_ = mlr::Array< std::shared_ptr<FOL_World> > ( nWorlds );
    bs_ = arr( nWorlds );
    for( uint w = 0; w < nWorlds; w++ )
    {
      // retrieve the facts of the belief state
      std::shared_ptr<FOL_World> fol = std::make_shared<FOL_World>();
      fol->init(FILE(folDescription.c_str()));
      auto n = bsGraph->elem(w);

      std::cout << "n:" << *n << std::endl;

      // add facts
      double probability = -1;

      for( auto nn : n->graph() )
      {
        StringA fact;

        for( auto f : nn->parents )
        {
          fact.append( f->keys.last() );
        }

        if( ! fact.empty() )
        {
          // tag this fact as not observable
          StringA notObservableFact; notObservableFact.append( notObservableTag );
          for( auto s : fact ) notObservableFact.append( s );

          fol->addFact(notObservableFact);

          //std::cout << "fact:" << fact << std::endl;
          //std::cout << "notObservableFact:" << notObservableFact << std::endl;
        }
        else
        {
          probability = nn->get<double>();
          //std::cout << probability << std::endl;
        }
      }

      fol->reset_state();

      //std::cout << *fol << std::endl; // tmp
      folEngines_(w) = fol;
      bs_(w) = probability;
    }

    // check that the belief state sums to 1
    double total = 0;
    for( auto p : bs_ ) total += p;

    CHECK( total == 1.00, "wrong belief state definition, the total of the probabilities doesn't sum to 1" );
  }

  root_ = std::make_shared< POGraphNode >( folEngines_, bs_ );
}

void GraphSearchPlanner::solve()
{
  std::cout << "GraphSearchPlanner::solveSymbolically" << std::endl;

  buildGraph();

  /*dijkstra();
  extractSolutions();
  buildPolicy();*/

  /*Dijkstra solver( folEngines_ );
  policy_ = solver.solve( root_, terminals_ );*/

  Yens solver( folEngines_ );
  auto policies = solver.solve( root_, terminals_, 10 );

  policy_ = policies.front();
}

void GraphSearchPlanner::integrate( const Policy::ptr & policy )
{ 

}

Policy::ptr GraphSearchPlanner::getPolicy() const
{
  return policy_;
}

MotionPlanningOrder GraphSearchPlanner::getPlanningOrder() const
{
  MotionPlanningOrder po( getPolicy()->id() );

  //
  po.setParam( "type", "jointPath" );
  //

  return po;
}

void GraphSearchPlanner::saveGraphToFile( const std::string & filename )
{
  if( ! root_ )
  {
    return;
  }

  std::ofstream file;
  file.open( filename );

  GraphPrinter printer( file );
  printer.print( root_, terminals_ );

  file.close();
}

void GraphSearchPlanner::buildGraph()
{
  //checked_.insert( root_ );
  queue_.push( root_ );

  while( ! queue_.empty() )
  {
    auto current = queue_.front();
    queue_.pop();

    if( current->isTerminal() )
    {
      std::cout << "terminal for bs:" << current->bs() << std::endl;

      terminals_.push_back( current );
    }
    else
    {
      if( ! current->isExpanded() )
      {
        auto newNodes = current->expand();

        for( auto n : newNodes )
        {
          queue_.push( n );
        }

        if( queue_.size() % 50 == 0 )
        {
          std::cout << "queue_.size():" << queue_.size() << std::endl;
        }
      }
    }
  }

  std::cout << "Graph build:" << root_->graph().size() << " number of terminal nodes:" << terminals_.size() << std::endl;
}

void GraphSearchPlanner::yen( uint k )   // generates a set of policies
{

}

void GraphSearchPlanner::checkIntegrity()
{

}

//---------Yens--------------------//

Yens::Yens( const mlr::Array< std::shared_ptr<FOL_World> > & folEngines )
  : folEngines_( folEngines )
  , dijkstra_  ( folEngines )
{

}

static std::list< PolicyNode::ptr > serializeFrom( const PolicyNode::ptr & node )
{
  std::list< PolicyNode::ptr > nodes;

  nodes.push_back( node );

  for( auto n : node->children() )
  {
    auto newNodes = serializeFrom( n );
    nodes.insert( nodes.end(), newNodes.begin(), newNodes.end() );
  }

  return nodes;
}

static std::list< PolicyNode::ptr > serialize( const Policy::ptr & policy )
{
  return serializeFrom( policy->root() );
}

std::list< Policy::ptr > Yens::solve( const POGraphNode::ptr & root, const std::list < POGraphNode::ptr > & terminals, const uint k )
{
  root_ = root;

  std::list< Policy::ptr > policies;

  auto policy_0 = dijkstra_.solve( root, terminals );
  policies.push_back( policy_0 );

  auto lastPolicy = policy_0;
  for( auto l = 1; l < k; ++l )
  {
    // serialize the solution
    auto s_lastPolicy = serialize( lastPolicy );

    for( auto i = 0; i < s_lastPolicy.size(); ++i ) ///*auto sit = std::begin( s_lastPolicy ); sit != std::end( s_lastPolicy ); ++sit*/ ) // s is the spur node
    {
      auto spurNodeIt = s_lastPolicy.begin();
      std::advance( spurNodeIt, i );
      auto spurNode   = *spurNodeIt;
      auto rootPath = std::list< PolicyNode::ptr >( std::begin( s_lastPolicy ), spurNodeIt );

      for( auto previousPolicy : policies )
      {
        auto s_previousPolicy = serialize( previousPolicy );
        auto ithNodeIt = s_previousPolicy.begin();
        std::advance( ithNodeIt, i );
        auto previousRootPath = std::list< PolicyNode::ptr >( std::begin( s_previousPolicy ), ithNodeIt );

        if( rootPath == previousRootPath )
        {
          // Remove the links that are part of the previous shortest paths which share the same root path.
          //auto e = std::pair< uint, uint >{ (*ithNodeIt)->id(), (*(++ithNodeIt))->id() };
          //dijkstra_.blackListEdge( e );
        }

        for( auto n : rootPath )
        {
          
        }
      }



      // reset
      dijkstra_.resetBlackList();
    }
  }

  return policies;
}

//---------Dijkstra-----------------//

Dijkstra::Dijkstra( const mlr::Array< std::shared_ptr<FOL_World> > & folEngines )
  : folEngines_( folEngines )
{

}

Policy::ptr Dijkstra::solve( const POGraphNode::ptr & root, const std::list < POGraphNode::ptr > & terminals )
{
  root_ = root;

  dijkstra( terminals );
  extractSolutions();
  buildPolicy();

  return policy_;
}

void Dijkstra::blackListEdge( const std::pair< uint, uint > & e )
{
  edgeBlackList_.push_back( e );
}

void Dijkstra::resetBlackList()
{
  edgeBlackList_.clear();
}

void Dijkstra::dijkstra( const std::list < POGraphNode::ptr > & terminals )
{
  std::cout << "GraphSearchPlanner::dijkstra.." << std::endl;

  expectedReward_ = std::vector< double >( root_->graph().size(), -1000 ); // distance from root to vertex[i]

  auto comp = [ & ]( const POGraphNode::ptr & a, const POGraphNode::ptr & b ) -> bool
  {
    return expectedReward_[ a->id() ] > expectedReward_[ b->id() ];
  };

  std::priority_queue< POGraphNode::ptr, std::vector< POGraphNode::ptr >, decltype( comp ) > Q( comp );

  // expected reward up to terminal nodes
  // add terminal nodes to Q
  //for( uint i = 0; i < 30; ++i )
  {

  // go from leafs to root
  for( auto v : terminals )
  {
    expectedReward_[ v->id() ] = 0; // all rewards negative
    Q.push( v );
  }

  // algorithm
  while( ! Q.empty() )
  {
    auto u = Q.top();
    Q.pop();

    for( auto parent : u->parents() )
    {
      double one = 0;

      one += u->p();
      auto alternativeReward = u->p() * ( expectedReward_[ u->id() ] - 1 ); // p->(u,v)

      for( auto v : u->andSiblings() )
      {
        one += v->p();
        alternativeReward += v->p() * ( expectedReward_[ v->id() ] - 1 );
      }

      CHECK( fabs( 1.0 - one ) < eps(), "corruption in probability computation!!" );

      if( alternativeReward > expectedReward_[ parent->id() ] )
      {
        expectedReward_[ parent->id() ] = alternativeReward;
        Q.push( parent );
      }
    }
  }

  }

  std::cout << "GraphSearchPlanner::dijkstra.. end" << std::endl;
}

void Dijkstra::extractSolutions()
{
  bestFamily_ = std::vector< int >( root_->graph().size(), -1 );
  parents_    = std::vector< POGraphNode::ptr >( root_->graph().size() );

  extractSolutionFrom( root_ );
}

void Dijkstra::extractSolutionFrom( const POGraphNode::ptr & node )
{
  double rewardFromNode = expectedReward_[ node->id() ];

  for( auto i = 0; i < node->families().size(); ++i )
  {
    auto f = node->families()( i );

    double familyReward = 0;

    for( auto c : f )
    {
      familyReward += c->p() * expectedReward_[ c->id() ];
    }

    if( familyReward >= rewardFromNode )
    {
      bestFamily_[ node->id() ] = i;

      for( auto c : f )
      {
        parents_[ c->id() ] = node;

        if( ! c->isTerminal() )
        {
          extractSolutionFrom( c );
        }
      }
    }
  }

  CHECK( bestFamily_[ node->id() ] != -1, "" );
}

void Dijkstra::buildPolicy()
{
  // convert to a policy object
  policy_ = std::make_shared< Policy >();
  policy_->init( root_->N() );

  buildPolicyFrom( root_ );
}

void Dijkstra::buildPolicyFrom( const POGraphNode::ptr & node )
{
  PolicyNode::ptr policyNode = std::make_shared< PolicyNode >();

  if( node->isRoot() )
  {
    policyNode->setTime( 0 );
    policyNode->setState( node->folStates(), node->bs() );

    policy_->setRoot( policyNode );
    policy_->setExpectedSymReward( expectedReward_[ node->id() ] );
  }
  else
  {
    // set parent
    auto graphParent = parents_[ node->id() ];
    auto parent = PO2Policy_[ graphParent ];
    policyNode->setParent( parent );

    // get action graph ( have to be reconstructed with the right action! since it is a graph!!)
    uint a = node->getLeadingActionFrom( graphParent );

    auto parentStates = graphParent->folStates(); // start states
    mlr::Array< std::shared_ptr<Graph> > resultStates( parentStates.d0 );
    for( auto w = 0; w < node->N(); ++w )
    {
      if( node->bs()( w ) > eps() )
      {
        auto fol = folEngines_( w );
        auto startState = parentStates( w );
        fol->setState( startState.get() );
        auto action = fol->get_actions()[ a ];
        fol->transition( action );
        auto resultState = fol->createStateCopy();
        resultStates( w ).reset( resultState );
      }
    }
    policyNode->setState( resultStates, node->bs() );

    // add child to parent
    policyNode->setTime( parent->time() + 1 );

    parent->addChild( policyNode );
    parent->setNextAction( node->getLeadingActionFromStr( graphParent ) );

    if( node->isTerminal() )
    {
      policy_->addLeaf( policyNode );
    }
  }

  // set node data
  policyNode->setId( node->id() );
  policyNode->setDifferentiatingFact( node->differentiatingFacts() );
  policyNode->setP( node->pHistory() );

  // save correspondance
  PO2Policy_[ node ] = policyNode;
  Policy2PO_[ policyNode ] = node;

  if( ! node->isTerminal() )
  {
    auto bestFamily = node->families()( bestFamily_[ node->id() ] );
    for( auto c : bestFamily )
    {
      buildPolicyFrom( c );
    }
  }
}

}
