#include <po_djikstra.h>
#include <queue>

namespace tp
{

//---------Dijkstra-----------------//

Dijkstra::Dijkstra( const mlr::Array< std::shared_ptr<FOL_World> > & folEngines )
  : folEngines_( folEngines )
{

}

Policy::ptr Dijkstra::solve( const POGraph::ptr & graph, const POGraphNode::ptr & from, GraphEdgeRewards::ptr mask )
{
  graph_ = graph;

  if( ! mask )
  {
    mask = std::make_shared< GraphEdgeRewards >( graph );
  }

  dijkstra( graph_->terminals(), mask );
  extractSolutionFrom( from );
  buildPolicy( from );

  return policy_;
}

void Dijkstra::dijkstra( const std::list < POGraphNode::ptr > & terminals, GraphEdgeRewards::ptr mask )
{
  std::cout << "GraphSearchPlanner::dijkstra.." << std::endl;

  expectedReward_ = std::vector< double >( graph_->size(), m_inf() ); // distance from root to vertex[i]

  auto comp = [ & ]( const POGraphNode::ptr & a, const POGraphNode::ptr & b ) -> bool
  {
    return expectedReward_[ a->id() ] > expectedReward_[ b->id() ];
  };

  bestFamily_ = std::vector< int >( graph_->size(), -1 );
  parents_    = std::vector< POGraphNode::ptr >( graph_->size() );
  std::priority_queue< POGraphNode::ptr, std::vector< POGraphNode::ptr >, decltype( comp ) > Q( comp );

  // expected reward up to terminal nodes
  // add terminal nodes to Q
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
      bool isImpossible = false;
      const auto r = mask->reward( parent->id(), u->id() );
      isImpossible = isImpossible || r <= m_inf();

      double one = 0;

      one += u->p();
      auto alternativeReward = u->p() * ( expectedReward_[ u->id() ] + r ); // p->(u,v)

      for( auto v : u->andSiblings() )
      {
        const auto r = mask->reward( parent->id(), v->id() );
        isImpossible = isImpossible || r <= m_inf();

        one += v->p();
        alternativeReward += v->p() * ( expectedReward_[ v->id() ] + r );
      }

      if( isImpossible ) alternativeReward = m_inf();

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

void Dijkstra::extractSolutionFrom( const POGraphNode::ptr & node )
{
  std::cout << "extract solution from:" << node->id() << std::endl;

  double rewardFromNode = expectedReward_[ node->id() ];

  for( auto i = 0; i < node->families().size(); ++i )
  {
    auto f = node->families()( i );

    double familyReward = 0;

    for( auto c : f )
    {
      familyReward += c->p() * expectedReward_[ c->id() ];
    }

    if( familyReward >= 1 + rewardFromNode )
    {
      bestFamily_[ node->id() ] = i;

      for( auto c : f )
      {
        parents_[ c->id() ] = node;

        if( ! c->isTerminal() && node->id() != c->id() )
        {
          extractSolutionFrom( c );
        }
      }
    }
  }

  CHECK( node->isTerminal() || bestFamily_[ node->id() ] != -1, "" );
}

void Dijkstra::buildPolicy( const POGraphNode::ptr & from )
{
  // clear last correspondance
  PO2Policy_.clear();
  Policy2PO_.clear();

  if( expectedReward_[ from->id() ] == m_inf() )
  { // no solution has been found
    policy_.reset();
  }
  else
  {
    // convert to a policy object
    policy_ = std::make_shared< Policy >();
    policy_->init( from->N() );

    buildPolicyFrom( from, from );
  }
}

void Dijkstra::buildPolicyFrom( const POGraphNode::ptr & node, const POGraphNode::ptr & start )
{
  PolicyNode::ptr policyNode = std::make_shared< PolicyNode >();

  if( node->id() == start->id() )
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
      buildPolicyFrom( c, start );
    }
  }
}

}
